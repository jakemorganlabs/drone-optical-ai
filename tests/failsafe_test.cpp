// Part 7 failsafe state-machine unit tests.
// Walks BOOT -> IDLE -> ARMED -> AUTONOMOUS, vets allow_autonomy under fresh
// / stale / link-loss / recover / kill / low-battery / backhaul-loss cases.
#include "control/failsafe.hpp"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

static int failures = 0;
#define CHECK(cond) do { \
    if(!(cond)) { std::cerr << "CHECK FAILED: " << #cond \
        << " at " << __FILE__ << ":" << __LINE__ << "\n"; ++failures; } \
} while(0)

using clk = std::chrono::steady_clock;

// Helper: a pose stamped `ms` milliseconds in the past (negative ms => future).
static Pose make_pose(int age_ms) {
    Pose p;
    p.timestamp = clk::now() + std::chrono::milliseconds(age_ms);
    return p;
}

static void test_boot_to_autonomy() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::RETURN_ALONG_TRACK;
    cfg.low_batt_pct = 20.0f;

    Failsafe f(cfg);
    CHECK(f.state() == Failsafe::BOOT);
    CHECK(std::string(f.state_name()) == "BOOT");

    CHECK(f.request_idle());
    CHECK(f.state() == Failsafe::IDLE);
    CHECK(std::string(f.state_name()) == "IDLE");

    // Sanity: invalid transitions must be rejected.
    CHECK(!f.request_autonomy());   // can't go IDLE -> AUTONOMOUS
    CHECK(!f.request_teleop());     // can't go IDLE -> TELEOP

    CHECK(f.request_arm());
    CHECK(f.state() == Failsafe::ARMED);

    CHECK(f.request_autonomy());
    CHECK(f.state() == Failsafe::AUTONOMOUS);

    // Fresh pose (now) in AUTONOMOUS -> allow_autonomy true.
    Pose fresh = make_pose(0);
    CHECK(f.allow_autonomy(fresh) == true);

    // Stale pose (200ms old) -> false.
    Pose stale = make_pose(-200);
    CHECK(f.allow_autonomy(stale) == false);

    // Future pose is "fresh" (age < 100ms) -> true. Sanity check on the path.
    Pose future = make_pose(50);
    CHECK(f.allow_autonomy(future) == true);
}

static void test_tether_loss_recover() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::RETURN_ALONG_TRACK;
    cfg.low_batt_pct = 20.0f;
    Failsafe f(cfg);

    f.request_idle();
    f.request_arm();
    f.request_autonomy();
    CHECK(f.state() == Failsafe::AUTONOMOUS);

    // allow_autonomy true with fresh pose before tether loss.
    Pose fresh = make_pose(0);
    CHECK(f.allow_autonomy(fresh) == true);

    // Tether loss -> LINK_LOSS. allow_autonomy must be false (state != AUTONOMOUS).
    f.on_tether_loss();
    CHECK(f.state() == Failsafe::LINK_LOSS);
    CHECK(std::string(f.state_name()) == "LINK_LOSS");
    CHECK(f.allow_autonomy(fresh) == false);

    // Recover -> AUTONOMOUS again; fresh pose -> true.
    f.on_tether_recover();
    CHECK(f.state() == Failsafe::AUTONOMOUS);
    CHECK(f.allow_autonomy(fresh) == true);
}

static void test_kill_and_fault() {
    Failsafe::Config cfg;
    Failsafe f(cfg);
    f.request_idle();
    f.request_arm();
    f.request_autonomy();

    CHECK(!f.kill_requested());
    f.on_kill();
    CHECK(f.state() == Failsafe::FAULT);
    CHECK(std::string(f.state_name()) == "FAULT");
    CHECK(f.kill_requested() == true);

    // Even with a fresh pose, autonomy is disallowed from FAULT.
    Pose fresh = make_pose(0);
    CHECK(f.allow_autonomy(fresh) == false);

    // Explicit reset clears the latch and drops back to IDLE.
    f.reset_fault();
    CHECK(f.state() == Failsafe::IDLE);
    CHECK(!f.kill_requested());
    CHECK(!f.allow_autonomy(fresh));
}

static void test_bad_cfc() {
    Failsafe f;
    f.request_idle();
    f.request_arm();
    f.request_autonomy();

    f.on_bad_cfc();
    CHECK(f.state() == Failsafe::FAULT);
    // nan_seen_ is sticky: even after state is moved off FAULT manually, the
    // latch must keep autonomy off until reset_fault(). We can't move state
    // off FAULT through public API except reset_fault, so just confirm the
    // current state rejects autonomy.
    CHECK(f.allow_autonomy(make_pose(0)) == false);
}

static void test_low_battery_rtl() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::RETURN_ALONG_TRACK;  // -> RTL action path
    cfg.low_batt_pct = 20.0f;
    Failsafe f(cfg);

    f.request_idle();
    f.request_arm();
    f.request_autonomy();

    // Battery above threshold: no transition.
    f.on_low_battery(50.0f);
    CHECK(f.state() == Failsafe::AUTONOMOUS);

    // Battery below threshold: RTL (config action != LAND).
    f.on_low_battery(15.0f);
    CHECK(f.state() == Failsafe::RTL);
}

static void test_low_battery_landing() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::LAND;
    cfg.low_batt_pct = 20.0f;
    Failsafe f(cfg);

    f.request_idle();
    f.request_arm();
    f.request_autonomy();

    f.on_low_battery(15.0f);
    CHECK(f.state() == Failsafe::LANDING);
}

static void test_geofence_breach() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::RETURN_ALONG_TRACK;
    Failsafe f(cfg);

    f.request_idle();
    f.request_arm();
    f.request_autonomy();

    f.on_geofence_breach();
    CHECK(f.state() == Failsafe::RTL);

    // LAND config -> LANDING.
    Failsafe::Config cfg2;
    cfg2.link_loss_action = Failsafe::LAND;
    Failsafe f2(cfg2);
    f2.request_idle();
    f2.request_arm();
    f2.request_autonomy();
    f2.on_geofence_breach();
    CHECK(f2.state() == Failsafe::LANDING);
}

static void test_backhaul_no_change() {
    Failsafe::Config cfg;
    cfg.link_loss_action = Failsafe::RETURN_ALONG_TRACK;
    cfg.low_batt_pct = 20.0f;
    Failsafe f(cfg);

    f.request_idle();
    f.request_arm();
    f.request_autonomy();
    Pose fresh = make_pose(0);

    // Backhaul loss must NOT change the flight state.
    f.on_backhaul_loss();
    CHECK(f.state() == Failsafe::AUTONOMOUS);
    CHECK(f.allow_autonomy(fresh) == true);

    // Second backhaul event also harmless.
    f.on_backhaul_loss();
    CHECK(f.state() == Failsafe::AUTONOMOUS);
}

static void test_teleop_transitions() {
    Failsafe f;
    f.request_idle();
    f.request_arm();

    // ARMED -> TELEOP allowed; AUTONOMOUS -> TELEOP allowed.
    CHECK(f.request_teleop());
    CHECK(f.state() == Failsafe::TELEOP);

    // allow_autonomy must be false in TELEOP (only AUTONOMOUS allowed).
    CHECK(f.allow_autonomy(make_pose(0)) == false);

    // From TELEOP -> IDLE via disarm.
    CHECK(f.request_disarm());
    CHECK(f.state() == Failsafe::IDLE);
}

int main() {
    test_boot_to_autonomy();
    test_tether_loss_recover();
    test_kill_and_fault();
    test_bad_cfc();
    test_low_battery_rtl();
    test_low_battery_landing();
    test_geofence_breach();
    test_backhaul_no_change();
    test_teleop_transitions();

    if (failures == 0) {
        std::cout << "All failsafe tests passed.\n";
        return 0;
    }
    std::cerr << failures << " checks failed.\n";
    return 1;
}