#pragma once

// Part 7 — Failsafe state machine (cross-cutting).
//
// One implementation, consulted everywhere. It vets whether offboard autonomy
// is currently safe to actuate (the FcBridge calls `allow_autonomy()` before
// issuing any velocity setpoint); it owns link-loss / low-battery / geofence /
// kill behavior; it is *not* responsible for the actual HOLD/LAND/EMERGENCY
// command emission — the caller (FcBridge) is. This class only vets and
// tracks state.
//
// Threading: the FcBridge drives this from its own thread; no internal sync is
// required. State is just an enum + a couple of flags.

#include "pose.hpp"

#include <chrono>
#include <cstdio>

class Failsafe {
public:
    // ----- Configuration --------------------------------------------------
    enum LinkLossAction { HOVER, RETURN_ALONG_TRACK, LAND };

    struct Config {
        LinkLossAction link_loss_action = RETURN_ALONG_TRACK;
        float low_batt_pct = 20.0f;
        // Configured action when low battery or geofence breach fires. The
        // manual lists RTL and LANDING as distinct states; pick one based on
        // whether RTL is available in the current autopilot mode / position
        // fix. We expose both so callers (FcBridge / Pilot) can match.
    };

    // ----- States ---------------------------------------------------------
    // Match the manual's enum exactly (RTL and LANDING listed separately).
    enum State {
        BOOT,
        IDLE,
        ARMED,
        AUTONOMOUS,
        TELEOP,
        LINK_LOSS,
        RTL,
        LANDING,
        FAULT
    };

    // ----- Lifecycle ------------------------------------------------------
    Failsafe() = default;
    explicit Failsafe(const Config& cfg) : cfg_(cfg) {}

    // ----- Mode transitions (sanity-checked) ------------------------------
    // BOOT -> IDLE once the runtime is up.
    bool request_idle() {
        if (state_ != BOOT) return false;
        state_ = IDLE;
        return true;
    }

    // IDLE -> ARMED (operator arms the drone).
    bool request_arm() {
        if (state_ != IDLE) return false;
        state_ = ARMED;
        return true;
    }

    // ARMED -> AUTONOMOUS (offboard pilot takes over).
    bool request_autonomy() {
        if (state_ != ARMED) return false;
        state_ = AUTONOMOUS;
        return true;
    }

    // ARMED <-> TELEOP (operator manual override). Allowed from ARMED or AUTONOMOUS.
    bool request_teleop() {
        if (state_ != ARMED && state_ != AUTONOMOUS) return false;
        state_ = TELEOP;
        return true;
    }

    // From TELEOP or AUTONOMOUS, drop back to ARMED (offboard released).
    bool request_disarm() {
        if (state_ != ARMED && state_ != TELEOP && state_ != AUTONOMOUS) return false;
        state_ = IDLE;
        return true;
    }

    // ----- Event hooks ----------------------------------------------------
    // The big one: should the FcBridge issue the offboard setpoint this tick?
    // True iff state == AUTONOMOUS, pose is fresh (< 100 ms), and we have not
    // seen a CfC NaN / sustained safety-filter veto.
    bool allow_autonomy(const Pose& p) const {
        using namespace std::chrono;
        auto age = steady_clock::now() - p.timestamp;
        bool pose_fresh = age < milliseconds(100);
        return state_ == AUTONOMOUS && pose_fresh && !nan_seen_;
    }

    // Tether (fiber) loss — keep flying onboard autonomy + configured action.
    // If we were in AUTONOMOUS we drop to LINK_LOSS but continue commanding
    // via Part-4 onboard autonomy (the FcBridge keeps sending setpoints
    // driven by the CfC / fallback planner). TELEOP over fiber is no longer
    // possible, so TELEOP also degrades to LINK_LOSS.
    void on_tether_loss() {
        if (state_ == AUTONOMOUS || state_ == TELEOP) {
            log_event("tether_loss", state_name(), "LINK_LOSS");
            state_ = LINK_LOSS;
        }
    }

    // Tether recovered — resume autonomy from LINK_LOSS.
    void on_tether_recover() {
        if (state_ == LINK_LOSS) {
            state_ = AUTONOMOUS;
            log_event("tether_recover", "LINK_LOSS", "AUTONOMOUS");
        }
    }

    // Backhaul (Starlink) loss — no flight behavior change. Mesh / LLM goals
    // keep their last value. Logged but no transition.
    void on_backhaul_loss() {
        log_event("backhaul_loss", state_name(), state_name());
    }

    // CfC output NaN/invalid OR safety-filter veto sustained. Latch: we go to
    // FAULT and the caller falls back to the deterministic planner, then
    // HOLD/LAND. `nan_seen_` is sticky — caller must NOT re-enable autonomy
    // without explicit reset (e.g. a mode re-request after a hand-off).
    void on_bad_cfc() {
        nan_seen_ = true;
        log_event("bad_cfc", state_name(), "FAULT");
        state_ = FAULT;
    }

    // Operator kill over fiber. Highest priority, never gated by autonomy.
    // The FcBridge maps this straight to EMERGENCY_STOP (motors off).
    void on_kill() {
        log_event("kill", state_name(), "FAULT");
        state_ = FAULT;
        kill_requested_ = true;
    }
    bool kill_requested() const { return kill_requested_; }

    // Low battery. Below the configured threshold -> RTL or LANDING per the
    // configured action. We pick RTL here (the manual lists both; RTL implies
    // a position-fixed return that the FC's own RTL mode handles; if no home
    // point / no GPS, the FcBridge should re-issue LANDING instead).
    void on_low_battery(float pct) {
        if (pct < cfg_.low_batt_pct) {
            apply_return_or_land("low_battery");
        }
    }

    // Geofence breach — same handling: RTL or LANDING per config.
    void on_geofence_breach() {
        apply_return_or_land("geofence_breach");
    }

    // Allow caller to clear a latched fault after a verified hand-off
    // (e.g. operator confirmed deterministic planner is healthy and wants to
    // re-arm). Intentionally explicit.
    void reset_fault() {
        nan_seen_ = false;
        kill_requested_ = false;
        if (state_ == FAULT) state_ = IDLE;
    }

    // ----- Accessors ------------------------------------------------------
    State state() const { return state_; }

    const char* state_name() const {
        switch (state_) {
            case BOOT:        return "BOOT";
            case IDLE:        return "IDLE";
            case ARMED:       return "ARMED";
            case AUTONOMOUS:  return "AUTONOMOUS";
            case TELEOP:      return "TELEOP";
            case LINK_LOSS:   return "LINK_LOSS";
            case RTL:         return "RTL";
            case LANDING:     return "LANDING";
            case FAULT:       return "FAULT";
        }
        return "UNKNOWN";
    }

    const Config& config() const { return cfg_; }

private:
    void apply_return_or_land(const char* why) {
        // RTL if the configured link-loss action wants us to return; LANDING
        // if it wants us to land in place. HOVER maps to LANDING here for
        // low-batt/geofence since hovering on a dead battery is the wrong
        // default — callers can override by re-requesting a mode.
        const char* target = (cfg_.link_loss_action == LAND) ? "LANDING" : "RTL";
        log_event(why, state_name(), target);
        state_ = (cfg_.link_loss_action == LAND) ? LANDING : RTL;
    }

    void log_event(const char* evt, const char* from, const char* to) const {
        std::fprintf(stderr, "[failsafe] %s: %s -> %s\n", evt, from, to);
    }

    Config cfg_;
    State state_ = BOOT;
    bool nan_seen_ = false;
    bool kill_requested_ = false;
};