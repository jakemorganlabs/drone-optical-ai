// Phase 2: MAVSDK flight-controller bridge (FCBridge).
//
// Streams FC EKF position + attitude into the perception provider and translates
// NavigationCommand into MAVSDK offboard setpoints. The NED FC frame is mapped to
// the repo frame as (north, east, -down): repo X=north, Y=east, Z=up (down is the
// negative of MAVSDK's down_m).
//
// Build gate: this TU is only added to the pilot_main link line when MAVSDK is
// detected by the build system. The header has no MAVSDK dependency, so callers
// can reference FcBridge declarations without having MAVSDK installed.

#include "fc_bridge.hpp"

#include <chrono>
#include <iostream>
#include <utility>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;

// ---------------------------------------------------------------------------
// MavState: keeps MAVSDK plugin handles + connection objects out of the header.
// ---------------------------------------------------------------------------
struct FcBridge::MavState {
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    std::shared_ptr<Telemetry> telemetry;
    std::shared_ptr<Offboard>  offboard;
};

FcBridge::FcBridge(DronePoseProvider& provider, const Failsafe* failsafe)
    : provider_(provider), failsafe_(failsafe), mav_(std::make_unique<MavState>()) {}

FcBridge::~FcBridge() = default;  // NOLINT: declared after MavState is complete

bool FcBridge::connect(const std::string& url) {
    if (mav_->mavsdk.add_any_connection(url) != ConnectionResult::Success) {
        std::cerr << "[fc_bridge] failed to add connection " << url << "\n";
        return false;
    }
    auto sys = mav_->mavsdk.first_autopilot(5.0);
    if (!sys) {
        std::cerr << "[fc_bridge] no autopilot discovered on " << url << "\n";
        return false;
    }
    mav_->telemetry = std::make_shared<Telemetry>(*sys);
    mav_->offboard  = std::make_shared<Offboard>(*sys);

    // Position + velocity (NED): drives last_pos_ + last_pose_time_.
    mav_->telemetry->subscribe_position_velocity_ned(
        [this](Telemetry::PositionVelocityNed pv) {
            std::lock_guard<std::mutex> lk(pm_);
            // NED -> repo frame (north, east, -down).
            last_pos_ = Vec3(pv.position.north_m,
                             pv.position.east_m,
                             -pv.position.down_m);
            pos_ok_ = true;
            last_pose_time_ = std::chrono::steady_clock::now();
            push_if_ready();
        });

    // Attitude (Euler, deg): completes the Pose once both first arrive.
    mav_->telemetry->subscribe_attitude_euler(
        [this](Telemetry::EulerAngle e) {
            std::lock_guard<std::mutex> lk(pm_);
            last_att_ = Att{static_cast<float>(e.yaw_deg),
                            static_cast<float>(e.pitch_deg),
                            static_cast<float>(e.roll_deg)};
            att_ok_ = true;
            push_if_ready();
        });

    connected_ = true;
    return true;
}

void FcBridge::push_if_ready() {
    // Caller holds pm_.
    if (!pos_ok_ || !att_ok_) return;
    Pose p(last_pos_, last_att_.yaw, last_att_.pitch, last_att_.roll);
    p.timestamp = std::chrono::steady_clock::now();
    provider_.push_pose(p);
}

void FcBridge::ensure_offboard() {
    if (!mav_->offboard) return;
    if (!mav_->offboard->is_active()) {
        // MAVSDK requires a setpoint be armed before start() is called.
        Offboard::VelocityNedYaw zero{};
        mav_->offboard->set_velocity_ned(zero);
        Offboard::Result r = mav_->offboard->start();
        if (r != Offboard::Result::Success) {
            std::cerr << "[fc_bridge] offboard start failed: "
                      << static_cast<int>(r) << "\n";
        }
    }
}

void FcBridge::send(const NavigationCommand& cmd) {
    if (!connected_ || !mav_->offboard) return;

    auto pose_age_ms = [&] {
        std::lock_guard<std::mutex> lk(pm_);
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - last_pose_time_).count();
    };

    switch (cmd.type) {
    case NavigationCommand::VELOCITY_SETPOINT: {
        // Safety: gate autonomy on the failsafe + pose freshness (Part 3.3).
        Pose latest{};
        bool have_pose = provider_.get_latest_pose(latest);
        bool allowed = true;
        if (failsafe_ && have_pose) allowed = failsafe_->allow_autonomy(latest);
        if (!have_pose || !allowed || pose_age_ms() > 100) {
            // Collapse to a HOLD instead of issuing the velocity setpoint.
            Offboard::VelocityNedYaw z{};
            ensure_offboard();
            mav_->offboard->set_velocity_ned(z);
            return;
        }
        // repo frame -> NED for MAVSDK: down = -z.
        Offboard::VelocityNedYaw v{};
        v.north_m_s = cmd.velocity.x;
        v.east_m_s  = cmd.velocity.y;
        v.down_m_s  = -cmd.velocity.z;
        v.yaw_deg   = cmd.yaw;
        ensure_offboard();
        mav_->offboard->set_velocity_ned(v);
        break;
    }
    case NavigationCommand::HOLD: {
        Offboard::VelocityNedYaw z{};
        ensure_offboard();
        mav_->offboard->set_velocity_ned(z);
        break;
    }
    case NavigationCommand::LAND: {
        // Stop offboard and let the FC execute its land mode.
        if (mav_->offboard->is_active()) (void)mav_->offboard->stop();
        break;
    }
    case NavigationCommand::EMERGENCY_STOP: {
        // Never gated: cut motors immediately. Offboard is stopped; the Action
        // plugin's kill() (Part 3.3 manual reference) is wired here once Phase 3
        // pulls in the Action plugin on the same connection. For now we drop the
        // setpoint stream, which the FC reacts to per its failsafe policy.
        if (mav_->offboard->is_active()) (void)mav_->offboard->stop();
        break;
    }
    case NavigationCommand::POSITION_SETPOINT:
    default:
        // Not used by the onboard pilot yet; silently ignore rather than guess.
        break;
    }
}