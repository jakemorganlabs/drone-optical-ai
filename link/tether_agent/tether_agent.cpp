// Phase 4 scaffold: fiber-optic tether datalink agent.
//
// Single-thread-per-connection server driven by POSIX sockets + JSON-line
// framing. Not a real gRPC server yet. The wire format mirrors tether.proto
// field-by-field; see link/tether_agent/README.md for the exact JSON shape.
//
//   TODO(phase5): swap to gRPC + protobuf once deps are vendored.

#include "tether_agent.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/tcp.h>
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
// tiny JSON helpers (no external dep) — enough for the wire types in
// tether.proto. We intentionally do not pull nlohmann/json; the wire format
// is small and fixed.
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

// Serialize a DroneState to a single JSON line (no trailing newline).
// Object members are ordered to match tether.proto field numbers.
std::string to_json(const DroneState& s) {
    std::ostringstream o;
    o << "{\"drone_id\":" << escape_json(s.drone_id)
      << ",\"pose\":{\"x\":" << s.pose.x
      << ",\"y\":" << s.pose.y
      << ",\"z\":" << s.pose.z
      << ",\"yaw\":"   << s.pose.yaw
      << ",\"pitch\":" << s.pose.pitch
      << ",\"roll\":"  << s.pose.roll
      << "}"
      << ",\"battery\":" << s.battery
      << ",\"map_delta\":" << escape_json(s.map_delta_b64)
      << ",\"state\":" << static_cast<int>(s.state) << "}";
    return o.str();
}

// A minimal recursive-descent parser for the narrow subset of JSON we
// accept on the wire. Sizes are tiny (one command per line); clarity wins
// over speed here.
class JsonValue {
public:
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
        while (pos < src.size() && (src[pos] == ' ' || src[pos] == '\t' ||
               src[pos] == '\n' || src[pos] == '\r'))
            ++pos;
    }
    bool match(char c) { skip_ws(); if (pos < src.size() && src[pos] == c) { ++pos; return true; } return false; }
    void expect(char c) { if (!match(c)) throw std::runtime_error("json: expected"); }

    JsonValue parse() {
        skip_ws();
        if (pos >= src.size()) throw std::runtime_error("json: eof");
        char c = src[pos];
        if (c == '{') return parse_obj();
        if (c == '[') return parse_arr();
        if (c == '"') { JsonValue v; v.type = JsonValue::String; v.str = parse_string(); return v; }  // pos at opening quote
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') { pos += 4; JsonValue v; v.type = JsonValue::Null; return v; }
        return parse_number();
    }

    JsonValue parse_obj() {
        JsonValue v; v.type = JsonValue::Object;
        ++pos; // {
        if (match('}')) return v;
        for (;;) {
            skip_ws();
            // parse_string consumes the opening quote itself
            std::string key = parse_string();
            expect(':');
            JsonValue val = parse();
            v.obj.emplace_back(std::move(key), std::move(val));
            if (match(',')) continue;
            expect('}');
            break;
        }
        return v;
    }
    JsonValue parse_arr() {
        JsonValue v; v.type = JsonValue::Array;
        ++pos; // [
        if (match(']')) return v;
        for (;;) {
            JsonValue e = parse();
            v.arr.push_back(std::move(e));
            if (match(',')) continue;
            expect(']');
            break;
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
    // Assumes src[pos] == '"'. Consumes both quotes and returns contents.
    std::string parse_string() {
        std::string out;
        ++pos; // skip opening quote
        while (pos < src.size() && src[pos] != '"') {
            char c = src[pos];
            if (c == '\\' && pos + 1 < src.size()) {
                ++pos;
                char e = src[pos];
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
                out += c;
                ++pos;
            }
        }
        // skip closing quote
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

// Parse one JSON-line GroundCommand. On any parse error returns false.
bool parse_ground_command(const std::string& line, GroundCommand& out) {
    Parser p(line);
    JsonValue root;
    try { root = p.parse(); } catch (...) { return false; }
    if (root.type != JsonValue::Object) return false;

    out = GroundCommand{};
    // oneof c tags handled by presence of these keys
    if (const auto* g = find(root, "goal")) {
        if (g->type != JsonValue::Object) return false;
        out.kind = CmdKind::Goal;
        if (const auto* pose = find(*g, "pose")) {
            if (pose->type != JsonValue::Object) return false;
            out.goal.pose.x = get_num(*pose, "x", 0.0);
            out.goal.pose.y = get_num(*pose, "y", 0.0);
            out.goal.pose.z = get_num(*pose, "z", 0.0);
            out.goal.pose.yaw   = get_num(*pose, "yaw", 0.0);
            out.goal.pose.pitch = get_num(*pose, "pitch", 0.0);
            out.goal.pose.roll  = get_num(*pose, "roll", 0.0);
        }
        out.goal.id = get_str(*g, "id");
        return true;
    }
    if (const auto* t = find(root, "teleop")) {
        if (t->type != JsonValue::Object) return false;
        out.kind = CmdKind::Teleop;
        out.teleop.vx = static_cast<float>(get_num(*t, "vx", 0.0));
        out.teleop.vy = static_cast<float>(get_num(*t, "vy", 0.0));
        out.teleop.vz = static_cast<float>(get_num(*t, "vz", 0.0));
        out.teleop.yaw_rate = static_cast<float>(get_num(*t, "yaw_rate", 0.0));
        return true;
    }
    if (const auto* k = find(root, "kill")) {
        out.kind = CmdKind::Kill;
        if (k->type == JsonValue::Object) out.kill.reason = get_str(*k, "reason");
        return true;
    }
    if (const auto* m = find(root, "mode")) {
        if (m->type != JsonValue::Object) return false;
        out.kind = CmdKind::Mode;
        int v = static_cast<int>(get_num(*m, "mode", 0.0));
        // match ModeChange proto enum (proto3 default 0, then AUTONOMOUS=1,TELEOP=2,IDLE=3)
        switch (v) {
            case 1: out.mode.mode = FlightMode::AUTONOMOUS; break;
            case 2: out.mode.mode = FlightMode::TELEOP;      break;
            case 3: out.mode.mode = FlightMode::IDLE;       break;
            default: return false;
        }
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Socket helpers
// ---------------------------------------------------------------------------
bool set_nonblocking(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

ssize_t read_line(int fd, std::string& line, std::string& pending) {
    // Returns: >0 a complete line was returned in `line`
    //           0  peer closed cleanly
    //          -1  no complete line yet (EAGAIN/EWOULDBLOCK)
    //          -2  hard error / peer reset
    for (;;) {
        // try to find a newline in the pending buffer
        size_t nl = pending.find('\n');
        if (nl != std::string::npos) {
            line = pending.substr(0, nl);
            pending.erase(0, nl + 1);
            return 1;
        }
        char buf[1024];
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n > 0) { pending.append(buf, buf + n); continue; }
        if (n == 0) return 0;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return -1;
        return -2;
    }
}

ssize_t write_all(int fd, const std::string& s, size_t& off) {
    // Continues writing from `off`. Returns: 0 done, -1 partial (EAGAIN),
    // -2 hard error.
    while (off < s.size()) {
        ssize_t n = ::send(fd, s.data() + off, s.size() - off, 0);
        if (n > 0) { off += static_cast<size_t>(n); continue; }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return -1;
        return -2;
    }
    return 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------
struct TetherAgent::Impl {
    std::function<DroneState()>         src_cb;
    std::function<void(const GroundCommand&)> cmd_cb;
    std::function<void()>               loss_cb;
    std::function<void()>               recover_cb;

    std::atomic<bool> running{false};
    std::thread       worker;

    int  listen_fd = -1;
};

TetherAgent::TetherAgent() : impl_(new Impl) {
    ::signal(SIGPIPE, SIG_IGN);  // we handle send errors explicitly
}

TetherAgent::~TetherAgent() { stop(); }

void TetherAgent::set_telemetry_source(std::function<DroneState()> src) {
    impl_->src_cb = std::move(src);
}
void TetherAgent::on_ground_command(std::function<void(const GroundCommand&)> cb) {
    impl_->cmd_cb = std::move(cb);
}
void TetherAgent::on_tether_loss(std::function<void()> cb) {
    impl_->loss_cb = std::move(cb);
}
void TetherAgent::on_tether_recover(std::function<void()> cb) {
    impl_->recover_cb = std::move(cb);
}

bool TetherAgent::start(uint16_t port) {
    if (impl_->running) return true;

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;

    int yes = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
        ::close(fd); return false;
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd); return false;
    }
    if (::listen(fd, 1) < 0) {
        ::close(fd); return false;
    }
    if (!set_nonblocking(fd)) {
        ::close(fd); return false;
    }

    impl_->listen_fd = fd;
    impl_->running = true;
    impl_->worker = std::thread([this, port]() {
        worker_loop(port);
    });
    return true;
}

void TetherAgent::stop() {
    if (!impl_->running) return;
    impl_->running = false;
    if (impl_->listen_fd >= 0) { ::shutdown(impl_->listen_fd, SHUT_RDWR); ::close(impl_->listen_fd); impl_->listen_fd = -1; }
    if (impl_->worker.joinable()) impl_->worker.join();
}

void TetherAgent::worker_loop(uint16_t /*port*/) {
    using clock = std::chrono::steady_clock;

    // One client at a time; we re-accept on disconnect. This keeps the
    // scaffold small and matches the single-fiber-single-drone topology
    // of Option A. Multi-drone fleet handling lives in the ground
    // coordinator (Part 6), not here.
    //
    // `in_link_loss` survives across sessions so a reconnect after a drop
    // fires on_tether_recover on the first good frame (manual Part 5.3:
    // "On reconnect, re-sync map + resume goals cleanly").
    bool in_link_loss = false;
    while (impl_->running) {
        // accept (non-blocking poll)
        int cfd = -1;
        while (impl_->running) {
            sockaddr_in ca{};
            socklen_t clen = sizeof(ca);
            cfd = ::accept(impl_->listen_fd, reinterpret_cast<sockaddr*>(&ca), &clen);
            if (cfd >= 0) break;
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            // hard accept error: bail out of the agent
            impl_->running = false;
            return;
        }
        if (!impl_->running) { if (cfd >= 0) ::close(cfd); break; }

        set_nonblocking(cfd);

        // Heartbeat cadence and link-loss threshold (manual Part 5.3).
        constexpr auto HEARTBEAT     = std::chrono::milliseconds(200);
        constexpr auto LINK_LOSS_SIL = std::chrono::seconds(1);

        auto last_heartbeat = clock::now();
        auto last_rx        = clock::now();

        std::string rx_pending;   // partial inbound line buffer
        std::string tx_pending;   // outbound buffer for partial writes
        size_t      tx_off = 0;
        bool        tx_busy = false;

        // Session loop
        bool peer_closed = false;
        while (impl_->running && !peer_closed) {
            auto now = clock::now();

            // --- receive any inbound lines ---
            for (;;) {
                std::string line;
                ssize_t r = read_line(cfd, line, rx_pending);
                if (r > 0) {
                    last_rx = now;
                    if (in_link_loss) {
                        in_link_loss = false;
                        if (impl_->recover_cb) impl_->recover_cb();
                    }
                    if (line.empty()) continue;
                    GroundCommand gc;
                    if (parse_ground_command(line, gc)) {
                        if (impl_->cmd_cb) impl_->cmd_cb(gc);
                    }
                    continue;
                }
                if (r == -1) break;       // no more data right now
                // r == 0 (clean close) or -2 (error): drop the client
                peer_closed = true;
                break;
            }

            // --- heartbeat: enqueue a telemetry JSON line ---
            if (!peer_closed && now - last_heartbeat >= HEARTBEAT) {
                last_heartbeat = now;
                if (impl_->src_cb) {
                    DroneState st = impl_->src_cb();
                    std::string js = to_json(st) + "\n";
                    if (tx_busy) {
                        tx_pending.append(js);   // queue behind in-flight
                    } else {
                        tx_pending = std::move(js);
                        tx_off = 0;
                        tx_busy = true;
                    }
                }
            }

            // --- write any queued telemetry ---
            if (tx_busy) {
                ssize_t w = write_all(cfd, tx_pending, tx_off);
                if (w == 0) {
                    tx_busy = false;
                    tx_pending.clear();
                    tx_off = 0;
                } else if (w == -1) {
                    // partial; leave for next iteration
                } else {
                    // hard error -> treat as disconnect
                    peer_closed = true;
                }
            }

            // --- link-loss watchdog ---
            if (!in_link_loss && (now - last_rx) >= LINK_LOSS_SIL) {
                in_link_loss = true;
                if (impl_->loss_cb) impl_->loss_cb();
                // Per contract: do NOT disconnect or stop. Keep listening
                // and heartbeating; ground reconnect re-syncs here.
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        ::close(cfd);
        // On peer drop still in good link, fire loss cb so the failsafe
        // transitions to LINK_LOSS before a reconnect re-syncs.
        if (!in_link_loss) {
            in_link_loss = true;
            if (impl_->loss_cb) impl_->loss_cb();
        }
        // Loop back to accept() to wait for a reconnect; on reconnect the
        // first good RX transitions us out of link loss via recover_cb.
    }
}

}  // namespace dronectl

// ---------------------------------------------------------------------------
// Optional self-test binary: `make tether-test` builds this TU with
// -DTETHER_AGENT_TEST_MAIN to produce build/tether_agent.
// ---------------------------------------------------------------------------
#ifdef TETHER_AGENT_TEST_MAIN
#include <iostream>
int main() {
    using namespace dronectl;
    TetherAgent agent;
    std::atomic<bool> got_loss{false};
    std::atomic<bool> got_recover{false};

    agent.set_telemetry_source([] {
        DroneState s;
        s.drone_id = "drone-01";
        s.battery = 0.87f;
        s.state = FailsafeState::AUTONOMOUS;
        s.pose.x = 1.5; s.pose.y = -2.0; s.pose.z = 3.25; s.pose.yaw = 45.0;
        return s;
    });
    agent.on_ground_command([](const GroundCommand& c) {
        std::cout << "rx cmd kind=" << static_cast<int>(c.kind) << "\n";
    });
    agent.on_tether_loss([&] { got_loss = true; std::cout << "LINK_LOSS\n"; });
    agent.on_tether_recover([&] { got_recover = true; std::cout << "RECOVER\n"; });

    uint16_t port = 52000;
    if (const char* env = std::getenv("TETHER_PORT")) port = static_cast<uint16_t>(std::atoi(env));

    if (!agent.start(port)) {
        std::cerr << "failed to start tether agent on port " << port << "\n";
        return 1;
    }
    std::cout << "tether agent listening on 127.0.0.1:" << port << "\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    agent.stop();
    std::cout << "loss=" << got_loss << " recover=" << got_recover << "\n";
    return 0;
}
#endif