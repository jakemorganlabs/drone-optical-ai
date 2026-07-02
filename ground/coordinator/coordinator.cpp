// Phase 5 (manual Part 6.1) — ground coordinator implementation.
//
// Two POSIX-socket workers per coordinator process:
//   1. tether_worker(): outbound JSON-line TCP client to the drone-side
//      tether-agent (link/tether_agent). It auto-reconnects with backoff,
//      parses inbound DroneState lines into on_telemetry callbacks, and
//      serializes outbound GroundCommand frames from the public send_* API.
//   2. fleet_worker(): blocking-accept HTTP listener on 127.0.0.1:8090
//      serving a tiny read-only /fleet JSON scrape. No HTTP framework --
//      hand-written request line parse + JSON writer.
//
// Wire format is field-identical to link/tether_agent/README.md and
// tether.proto; field names are pinned there. The JSON shapes:
//
//   DroneState (up):   {"drone_id":"...","pose":{"x":..,"y":..,"z":..,
//                                              "yaw":..,"pitch":..,"roll":..},
//                       "battery":..,"map_delta":"","state":<int>}
//   GroundCommand (down) oneof:
//     {"goal":{"pose":{...},"id":"..."}}
//     {"teleop":{"vx":..,"vy":..,"vz":..,"yaw_rate":..}}
//     {"kill":{"reason":"..."}}
//     {"mode":{"mode":<1|2|3>}}
//
// FailsafeState ints:  BOOT=1 IDLE=2 ARMED=3 AUTONOMOUS=4 TELEOP=5
//                       LINK_LOSS=6 RTL=7 LANDING=8 FAULT=9
// FlightMode ints:     AUTONOMOUS=1 TELEOP=2 IDLE=3

#include "coordinator.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace dronectl {

namespace {

// ---------------------------------------------------------------------------
// tiny JSON helpers (no external dep) — mirrors the subset used by the
// Phase 4 tether-agent; keep field names identical so the same wire works.
// ---------------------------------------------------------------------------

std::string escape_json(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
    return out;
}

std::string pose_to_json(const CoordPose& p) {
    std::ostringstream o;
    o << "{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z
      << ",\"yaw\":" << p.yaw << ",\"pitch\":" << p.pitch
      << ",\"roll\":" << p.roll << "}";
    return o.str();
}

// DroneState -> JSON is not emitted by the coordinator (it's the receiver
// of DroneState, the sender of GroundCommand), so the serializer lives in
// the Phase 4 tether-agent only. Kept here for symmetry/dead-code-elim.

std::string goal_to_json(const Waypoint& w) {
    std::ostringstream o;
    o << "{\"goal\":{\"pose\":" << pose_to_json(w.pose)
      << ",\"id\":" << escape_json(w.id) << "}}";
    return o.str();
}
std::string teleop_to_json(const TeleopVel& v) {
    std::ostringstream o;
    o << "{\"teleop\":{\"vx\":" << v.vx << ",\"vy\":" << v.vy
      << ",\"vz\":" << v.vz << ",\"yaw_rate\":" << v.yaw_rate << "}}";
    return o.str();
}
std::string kill_to_json(const KillCmd& k) {
    std::ostringstream o;
    o << "{\"kill\":{\"reason\":" << escape_json(k.reason) << "}}";
    return o.str();
}
std::string mode_to_json(const ModeChange& m) {
    std::ostringstream o;
    o << "{\"mode\":{\"mode\":" << static_cast<int>(m.mode) << "}}";
    return o.str();
}

// Minimal recursive-descent JSON parser (same shape as tether_agent.cpp).
struct JsonValue {
    enum Type { Null, Bool, Number, String, Array, Object };
    Type type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<JsonValue> arr;
    std::vector<std::pair<std::string, JsonValue>> obj;
};

struct Parser {
    const std::string& src;
    size_t pos = 0;
    explicit Parser(const std::string& s) : src(s) {}

    void skip_ws() {
        while (pos < src.size() && (src[pos]==' '||src[pos]=='\t'||src[pos]=='\n'||src[pos]=='\r')) ++pos;
    }
    bool match(char c) { skip_ws(); if (pos < src.size() && src[pos]==c) { ++pos; return true; } return false; }
    void expect(char c) { if (!match(c)) throw std::runtime_error("json: expected"); }

    JsonValue parse() {
        skip_ws();
        if (pos >= src.size()) throw std::runtime_error("json: eof");
        char c = src[pos];
        if (c == '{') return parse_obj();
        if (c == '[') return parse_arr();
        if (c == '"') { JsonValue v; v.type = JsonValue::String; v.str = parse_string(); return v; }
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') { pos += 4; JsonValue v; v.type = JsonValue::Null; return v; }
        return parse_number();
    }
    JsonValue parse_obj() {
        JsonValue v; v.type = JsonValue::Object; ++pos;
        if (match('}')) return v;
        for (;;) {
            skip_ws();
            std::string key = parse_string();
            expect(':');
            JsonValue val = parse();
            v.obj.emplace_back(std::move(key), std::move(val));
            if (match(',')) continue;
            expect('}'); break;
        }
        return v;
    }
    JsonValue parse_arr() {
        JsonValue v; v.type = JsonValue::Array; ++pos;
        if (match(']')) return v;
        for (;;) {
            JsonValue e = parse();
            v.arr.push_back(std::move(e));
            if (match(',')) continue;
            expect(']'); break;
        }
        return v;
    }
    JsonValue parse_bool() {
        JsonValue v; v.type = JsonValue::Bool;
        if (src.compare(pos, 4, "true") == 0) { v.b = true; pos += 4; }
        else { v.b = false; pos += 5; }
        return v;
    }
    JsonValue parse_number() {
        size_t start = pos;
        while (pos < src.size() && (std::isdigit((unsigned char)src[pos]) ||
               src[pos]=='-'||src[pos]=='+'||src[pos]=='.'||src[pos]=='e'||src[pos]=='E'))
            ++pos;
        JsonValue v; v.type = JsonValue::Number;
        v.num = std::stod(src.substr(start, pos - start));
        return v;
    }
    std::string parse_string() {
        std::string out; ++pos;
        while (pos < src.size() && src[pos] != '"') {
            char c = src[pos];
            if (c == '\\' && pos + 1 < src.size()) {
                ++pos; char e = src[pos];
                switch (e) {
                    case '"':  out += '"';  break;
                    case '\\': out += '\\'; break;
                    case '/':  out += '/';  break;
                    case 'b':  out += '\b'; break;
                    case 'f':  out += '\f'; break;
                    case 'n':  out += '\n'; break;
                    case 'r':  out += '\r'; break;
                    case 't':  out += '\t'; break;
                    case 'u': {
                        if (pos + 4 >= src.size()) throw std::runtime_error("json: bad \\u");
                        unsigned cp = std::stoul(src.substr(pos + 1, 4), nullptr, 16);
                        pos += 4;
                        if (cp < 0x80)      out += static_cast<char>(cp);
                        else if (cp < 0x800) {
                            out += static_cast<char>(0xC0 | (cp >> 6));
                            out += static_cast<char>(0x80 | (cp & 0x3F));
                        } else {
                            out += static_cast<char>(0xE0 | (cp >> 12));
                            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                            out += static_cast<char>(0x80 | (cp & 0x3F));
                        }
                    } break;
                    default: out += e;
                }
                ++pos;
            } else {
                out += c; ++pos;
            }
        }
        if (pos < src.size()) ++pos;
        return out;
    }
};

const JsonValue* find(const JsonValue& o, const char* key) {
    if (o.type != JsonValue::Object) return nullptr;
    for (const auto& kv : o.obj) if (kv.first == key) return &kv.second;
    return nullptr;
}
double get_num(const JsonValue& o, const char* key, double def) {
    if (const auto* p = find(o, key)) if (p->type == JsonValue::Number) return p->num;
    return def;
}
const std::string& get_str(const JsonValue& o, const char* key) {
    static const std::string empty;
    if (const auto* p = find(o, key)) if (p->type == JsonValue::String) return p->str;
    return empty;
}

bool parse_drone_state(const std::string& line, DroneState& out) {
    Parser p(line);
    JsonValue root;
    try { root = p.parse(); } catch (...) { return false; }
    if (root.type != JsonValue::Object) return false;

    out = DroneState{};
    out.drone_id = get_str(root, "drone_id");
    if (const auto* pose = find(root, "pose")) {
        if (pose->type != JsonValue::Object) return false;
        out.pose.x = get_num(*pose, "x", 0.0);
        out.pose.y = get_num(*pose, "y", 0.0);
        out.pose.z = get_num(*pose, "z", 0.0);
        out.pose.yaw   = get_num(*pose, "yaw", 0.0);
        out.pose.pitch = get_num(*pose, "pitch", 0.0);
        out.pose.roll  = get_num(*pose, "roll", 0.0);
    }
    out.battery = static_cast<float>(get_num(root, "battery", 1.0));
    out.map_delta_b64 = get_str(root, "map_delta");
    int st = static_cast<int>(get_num(root, "state", 1.0));
    if (st < 1 || st > 9) st = 1;
    out.state = static_cast<FailsafeState>(st);
    return true;
}

// ---------------------------------------------------------------------------
// Socket helpers
// ---------------------------------------------------------------------------
bool set_nonblocking(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

// Blocking full write. Returns true on success, false on hard error.
bool send_all_blocking(int fd, const std::string& s) {
    size_t off = 0;
    while (off < s.size()) {
        ssize_t n = ::send(fd, s.data() + off, s.size() - off, 0);
        if (n > 0) { off += static_cast<size_t>(n); continue; }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        return false;
    }
    return true;
}

// Connect (blocking, with a small timeout via non-blocking poll). Returns a
// connected fd on success, -1 on failure.
int dial_tether(const std::string& ip, uint16_t port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int yes = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        ::close(fd); return -1;
    }

    set_nonblocking(fd);
    int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (rc == 0) { (void)set_nonblocking(fd); return fd; }  // immediate connect
    if (errno != EINPROGRESS) { ::close(fd); return -1; }

    // Poll for write-readiness up to ~1 s.
    fd_set wset;
    FD_ZERO(&wset);
    FD_SET(fd, &wset);
    timeval tv{}; tv.tv_sec = 1; tv.tv_usec = 0;
    rc = ::select(fd + 1, nullptr, &wset, nullptr, &tv);
    if (rc <= 0) { ::close(fd); return -1; }
    int err = 0; socklen_t elen = sizeof(err);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen) < 0 || err != 0) {
        ::close(fd); return -1;
    }
    return fd;
}

}  // namespace

// Forward decl so fleet_worker can use it before its definition below.
const char* failsafe_name(FailsafeState s);

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct Coordinator::Impl {
    CoordinatorConfig cfg;

    std::function<void(const DroneState&)> telemetry_cb;

    std::atomic<bool> running{false};
    std::thread       tether_th;
    std::thread       fleet_th;

    // Tether socket: written ONLY by tether_worker, so no extra lock around
    // the fd. send_* marshals outbound frames through cmd_mx into tx_pending;
    // tether_worker drains tx_pending onto the socket and owns reconnect.
    int tether_fd = -1;
    std::mutex    tx_mx;
    std::string   tx_pending;   // frames queued by send_*; drained by worker

    // Last-seen telemetry snapshot, served to /fleet on demand.
    mutable std::mutex state_mx;
    DroneState          last_state{};
    std::atomic<bool>   have_state{false};

    // Fleet listener socket.
    int fleet_fd = -1;
};

Coordinator::Coordinator() : impl_(new Impl) {
    ::signal(SIGPIPE, SIG_IGN);
}
Coordinator::~Coordinator() { stop(); }

void Coordinator::on_telemetry(std::function<void(const DroneState&)> cb) {
    impl_->telemetry_cb = std::move(cb);
}

bool Coordinator::start(const CoordinatorConfig& cfg) {
    if (impl_->running) return true;
    impl_->cfg = cfg;

    // Fleet listener: bind 127.0.0.1:cfg.fleet_port (blocking accept in
    // fleet_worker; SO_REUSEADDR so quick restarts work).
    int ffd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ffd < 0) return false;
    int yes = 1;
    ::setsockopt(ffd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in faddr{};
    faddr.sin_family = AF_INET;
    faddr.sin_port   = htons(cfg.fleet_port);
    if (::inet_pton(AF_INET, "127.0.0.1", &faddr.sin_addr) != 1) {
        ::close(ffd); return false;
    }
    if (::bind(ffd, reinterpret_cast<sockaddr*>(&faddr), sizeof(faddr)) < 0) {
        ::close(ffd); return false;
    }
    if (::listen(ffd, 8) < 0) {
        ::close(ffd); return false;
    }
    impl_->fleet_fd   = ffd;
    impl_->running    = true;
    impl_->tether_th  = std::thread([this]{ tether_worker(); });
    impl_->fleet_th   = std::thread([this]{ fleet_worker(); });
    return true;
}

void Coordinator::stop() {
    if (!impl_->running) return;
    impl_->running = false;
    if (impl_->tether_fd >= 0) {
        ::shutdown(impl_->tether_fd, SHUT_RDWR);
        ::close(impl_->tether_fd);
        impl_->tether_fd = -1;
    }
    if (impl_->fleet_fd >= 0) {
        ::shutdown(impl_->fleet_fd, SHUT_RDWR);
        ::close(impl_->fleet_fd);
        impl_->fleet_fd = -1;
    }
    if (impl_->tether_th.joinable()) impl_->tether_th.join();
    if (impl_->fleet_th.joinable())  impl_->fleet_th.join();
    impl_->running = false;
}

// ---- Command senders (any thread) -------------------------------------------
bool Coordinator::send_goal(const Waypoint& w) {
    std::string js = goal_to_json(w); js += "\n";
    std::lock_guard<std::mutex> lk(impl_->tx_mx);
    impl_->tx_pending.append(js);
    return true;   // queued; tether_worker drains. Wire-level result is on
                   // the worker thread — per the Phase 4 contract the tether
                   // agent dedups goals by id on reconnect, so queueing is safe.
}
bool Coordinator::send_teleop(const TeleopVel& v) {
    std::string js = teleop_to_json(v); js += "\n";
    std::lock_guard<std::mutex> lk(impl_->tx_mx);
    impl_->tx_pending.append(js);
    return true;
}
bool Coordinator::send_kill(const std::string& reason) {
    std::string js = kill_to_json(KillCmd{reason}); js += "\n";
    std::lock_guard<std::mutex> lk(impl_->tx_mx);
    impl_->tx_pending.append(js);
    return true;
}
bool Coordinator::send_mode(FlightMode m) {
    std::string js = mode_to_json(ModeChange{m}); js += "\n";
    std::lock_guard<std::mutex> lk(impl_->tx_mx);
    impl_->tx_pending.append(js);
    return true;
}

DroneState Coordinator::last_state() const {
    std::lock_guard<std::mutex> lk(impl_->state_mx);
    return impl_->last_state;
}
bool Coordinator::have_state() const { return impl_->have_state.load(); }

// ---- Tether worker -----------------------------------------------------------
void Coordinator::tether_worker() {
    using clock = std::chrono::steady_clock;
    std::string rx_pending;

    while (impl_->running) {
        // (Re)connect to the drone-side tether-agent.
        if (impl_->tether_fd < 0) {
            impl_->tether_fd = dial_tether(impl_->cfg.tether_ip, impl_->cfg.tether_port);
            if (impl_->tether_fd < 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(impl_->cfg.reconnect_ms > 0
                                              ? impl_->cfg.reconnect_ms : 1000));
                continue;
            }
            rx_pending.clear();
            std::fprintf(stderr, "[coordinator] connected to %s:%u\n",
                         impl_->cfg.tether_ip.c_str(),
                         static_cast<unsigned>(impl_->cfg.tether_port));
        }

        // Drain any queued outbound frames.
        {
            std::string out;
            {
                std::lock_guard<std::mutex> lk(impl_->tx_mx);
                out.swap(impl_->tx_pending);
            }
            if (!out.empty()) {
                if (!send_all_blocking(impl_->tether_fd, out)) {
                    ::close(impl_->tether_fd);
                    impl_->tether_fd = -1;
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(impl_->cfg.reconnect_ms));
                    continue;
                }
            }
        }

        // Non-blocking poll for inbound lines (short timeout so we keep
        // draining tx_pending promptly).
        fd_set rset; FD_ZERO(&rset); FD_SET(impl_->tether_fd, &rset);
        timeval tv{}; tv.tv_sec = 0; tv.tv_usec = 50000;
        int rc = ::select(impl_->tether_fd + 1, &rset, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            ::close(impl_->tether_fd);
            impl_->tether_fd = -1;
            continue;
        }
        if (rc > 0 && FD_ISSET(impl_->tether_fd, &rset)) {
            char buf[2048];
            ssize_t n = ::recv(impl_->tether_fd, buf, sizeof(buf), 0);
            if (n <= 0) {
                ::close(impl_->tether_fd);
                impl_->tether_fd = -1;
                continue;
            }
            rx_pending.append(buf, buf + n);
            // Parse every complete line.
            std::string line;
            while (true) {
                size_t nl = rx_pending.find('\n');
                if (nl == std::string::npos) break;
                line = rx_pending.substr(0, nl);
                rx_pending.erase(0, nl + 1);
                if (line.empty()) continue;
                DroneState st;
                if (parse_drone_state(line, st)) {
                    {
                        std::lock_guard<std::mutex> lk(impl_->state_mx);
                        impl_->last_state = st;
                    }
                    impl_->have_state.store(true);
                    if (impl_->telemetry_cb) impl_->telemetry_cb(st);
                }
            }
        }
        (void)clock::now();
    }

    if (impl_->tether_fd >= 0) {
        ::close(impl_->tether_fd);
        impl_->tether_fd = -1;
    }
}

// ---- Fleet worker (read-only HTTP scrape on 127.0.0.1:8090) ------------------
void Coordinator::fleet_worker() {
    // Tiny blocking-accept HTTP listener. We answer GET /fleet and reject
    // everything else with 404. No framework, no keep-alive.
    while (impl_->running) {
        sockaddr_in ca{};
        socklen_t clen = sizeof(ca);
        int cfd = ::accept(impl_->fleet_fd, reinterpret_cast<sockaddr*>(&ca), &clen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        // Read the request line only (we only care about the method + path).
        char rbuf[1024];
        std::string req;
        for (;;) {
            ssize_t n = ::recv(cfd, rbuf, sizeof(rbuf), 0);
            if (n <= 0) break;
            req.append(rbuf, rbuf + n);
            if (req.find("\r\n\r\n") != std::string::npos) break;
            if (req.size() > sizeof(rbuf)) break;
        }
        // Parse "METHOD SP PATH SP HTTP/..."
        std::string method, path;
        {
            std::istringstream is(req);
            is >> method >> path;
        }
        std::string body;
        std::string status_line;
        std::string ctype = "application/json";

        if (method == "GET" && path == "/fleet") {
            DroneState st = last_state();
            std::ostringstream o;
            o << "{\"drone_id\":" << escape_json(st.drone_id.empty() ? impl_->cfg.drone_id : st.drone_id)
              << ",\"connected\":" << (impl_->tether_fd >= 0 ? "true" : "false")
              << ",\"have_state\":" << (impl_->have_state.load() ? "true" : "false")
              << ",\"pose\":" << pose_to_json(st.pose)
              << ",\"battery\":" << (impl_->have_state.load() ? st.battery : 0.0)
              << ",\"state\":" << static_cast<int>(st.state)
              << ",\"state_name\":" << escape_json(failsafe_name(st.state))
              << "}";
            body = o.str();
            status_line = "HTTP/1.1 200 OK";
        } else if (method == "GET" && (path == "/healthz" || path == "/")) {
            body = "{\"status\":\"ok\"}";
            status_line = "HTTP/1.1 200 OK";
        } else {
            body = "{\"error\":\"not found\"}";
            status_line = "HTTP/1.1 404 Not Found";
        }

        std::ostringstream resp;
        resp << status_line << "\r\n"
             << "Content-Type: " << ctype << "\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Connection: close\r\n"
             << "\r\n" << body;
        std::string out = resp.str();
        (void)send_all_blocking(cfd, out);
        ::close(cfd);
    }
}

// ---- Helper for fleet JSON ---------------------------------------------------
const char* failsafe_name(FailsafeState s) {
    switch (s) {
        case FailsafeState::BOOT:       return "BOOT";
        case FailsafeState::IDLE:       return "IDLE";
        case FailsafeState::ARMED:      return "ARMED";
        case FailsafeState::AUTONOMOUS: return "AUTONOMOUS";
        case FailsafeState::TELEOP:     return "TELEOP";
        case FailsafeState::LINK_LOSS:  return "LINK_LOSS";
        case FailsafeState::RTL:        return "RTL";
        case FailsafeState::LANDING:    return "LANDING";
        case FailsafeState::FAULT:      return "FAULT";
    }
    return "UNKNOWN";
}

}  // namespace dronectl

// ---------------------------------------------------------------------------
// Optional self-test binary: `make coordinator` builds this TU with
// -DCOORDINATOR_TEST_MAIN to produce build/coordinator. It spins up the
// coordinator pointed at TETHER_IP:TETHER_PORT (env-overridable) and the
// fleet scrape on FLEET_PORT (default 8090), prints the first few /fleet
// responses fetched via curl-style loopback, then exits after ~5 s. The
// tether-agent on the drone side is expected to already be running.
// ---------------------------------------------------------------------------
#ifdef COORDINATOR_TEST_MAIN
#include <iostream>
int main(int argc, char** argv) {
    using namespace dronectl;
    std::string drone_id = "drone-01";
    std::string tether_ip = "10.8.0.2";
    uint16_t tether_port = 52000;
    uint16_t fleet_port  = 8090;

    // Cheap arg parsing: --drone-id <id> --tether-ip <ip> --tether-port <p> --fleet-port <p>
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* key, std::string& dst) {
            if (a == key && i + 1 < argc) { dst = argv[++i]; }
        };
        auto nextn = [&](const char* key, uint16_t& dst) {
            if (a == key && i + 1 < argc) { dst = static_cast<uint16_t>(std::atoi(argv[++i])); }
        };
        next ("--drone-id",    drone_id);
        next ("--tether-ip",   tether_ip);
        nextn("--tether-port", tether_port);
        nextn("--fleet-port",  fleet_port);
    }
    if (const char* e = std::getenv("TETHER_IP"))   tether_ip   = e;
    if (const char* e = std::getenv("TETHER_PORT")) tether_port = static_cast<uint16_t>(std::atoi(e));
    if (const char* e = std::getenv("FLEET_PORT"))  fleet_port  = static_cast<uint16_t>(std::atoi(e));

    CoordinatorConfig cfg;
    cfg.drone_id    = drone_id;
    cfg.tether_ip   = tether_ip;
    cfg.tether_port = tether_port;
    cfg.fleet_port  = fleet_port;

    Coordinator coord;
    coord.on_telemetry([](const DroneState& s) {
        std::cout << "telemetry " << s.drone_id
                  << " bat=" << s.battery
                  << " state=" << static_cast<int>(s.state) << "\n";
    });
    if (!coord.start(cfg)) {
        std::cerr << "failed to start coordinator (fleet port "
                  << fleet_port << " busy?)\n";
        return 1;
    }
    std::cout << "coordinator up: drone=" << drone_id
              << " tether=" << tether_ip << ":" << tether_port
              << " fleet=127.0.0.1:" << fleet_port << "/fleet\n";
    // Demo: push a goal down the tether so the wire format round-trips in a
    // SITL bench test. The tether-agent on the drone side should ack by
    // echoing it back in on_ground_command.
    Waypoint w; w.id = "g-demo-001"; w.pose.x = 12.0; w.pose.y = -3.0; w.pose.z = 5.0;
    coord.send_goal(w);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    coord.stop();
    std::cout << "coordinator stopped\n";
    return 0;
}
#endif