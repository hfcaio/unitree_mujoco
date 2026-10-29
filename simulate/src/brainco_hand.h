#pragma once

// BrainCo Revo2 dexterous hand, as the real G1 exposes it through
// unitreerobotics/brainco_hand_service (main.cpp): one pair of topics per hand,
//
//   rt/brainco/<side>/cmd    unitree_go::msg::dds_::MotorCmds_    (6 cmds)
//   rt/brainco/<side>/state  unitree_go::msg::dds_::MotorStates_  (6 states, 100 Hz)
//
// in the finger order [Thumb, ThumbAux, Index, Middle, Ring, Pinky]. Everything is normalized:
//   cmd.q   in [0, 1]: 0 = open, 1 = closed (the finger's full range)
//   cmd.dq  in [0, 1]: speed as a fraction of the finger's maximum speed (the service starts at 1)
//   state.q in [0, 1], state.dq in [-1, 1] (fraction of max speed, + = closing),
//   state.tau_est in [-1, 1] (motor current / max; here actuator force / force limit, + = closing).
// Simulator additions: state.q_raw / dq_raw hold the joint angle (rad) and velocity (rad/s), and
//   rt/brainco/<side>/touch  unitree_hg::msg::dds_::PressSensorState_
// publishes the fingertip tactile pads (the real hand service does not publish touch data):
//   pressure[0..4] = normal force (N), pressure[5..9] = tangential force magnitude (N),
//   both in the order [Thumb, Index, Middle, Ring, Pinky]; pressure[10..11] and temperature = 0.
//
// The hand model is unitree_robots/components/brainco_revo2/<side>.xml attached with a prefix
// (see robot_compose.h); this component finds its elements by prefixed name:
// actuators <prefix>{thumb, thumb_aux, index, middle, ring, pinky} (position servos whose ctrlrange
// is the joint range), sensors <prefix><actuator>_pos / _vel / _force, <prefix><finger>_touch
// (touch sensor: normal force) and optionally <prefix><finger>_contact (contact sensor with
// data="found force normal tangent" on the same pad site: tangential force).
//
// YAML params: side (left|right, required), prefix (default "<side>_"), topic (default
// "rt/brainco/<side>"), hz (state rate, default 100), touch (default true),
// max_velocity (6 values, rad/s, default from the Revo2 URDF).

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/PressSensorState_.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include "bridge_component.h"

class BraincoHandComponent : public BridgeComponent
{
    using Cmd_t = unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>;
    using State_t = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>;
    using Touch_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::PressSensorState_>;

    static constexpr int kMotors = 6;
    static constexpr int kFingers = 5;
    static constexpr double kBridgeHz = 1000.0;

public:
    BraincoHandComponent(mjModel *model, mjData *data, const YAML::Node &params) : BridgeComponent(model, data)
    {
        static const char *kActuators[kMotors] = {"thumb", "thumb_aux", "index", "middle", "ring", "pinky"};
        static const char *kTouch[kFingers] = {"thumb", "index", "middle", "ring", "pinky"};
        // Revo2 URDF (revo2_description v1.1.1) joint velocity limits, rad/s
        std::array<double, kMotors> vmax = {2.5303, 2.6175, 2.2685, 2.2685, 2.2685, 2.2685};

        if (!params["side"]) {
            throw std::invalid_argument("brainco_hand: param `side` (left or right) is required");
        }
        auto side = params["side"].as<std::string>();
        auto prefix = params["prefix"] ? params["prefix"].as<std::string>() : side + "_";
        auto topic = params["topic"] ? params["topic"].as<std::string>() : "rt/brainco/" + side;
        if (params["hz"]) {
            div_ = static_cast<int>(std::lround(kBridgeHz / params["hz"].as<double>()));
        }
        if (div_ < 1) {
            throw std::invalid_argument("brainco_hand: hz must be <= 1000");
        }
        if (params["max_velocity"]) {
            vmax = params["max_velocity"].as<std::array<double, kMotors>>();
        }
        bool touch = params["touch"] ? params["touch"].as<bool>() : true;

        for (int i = 0; i < kMotors; i++) {
            auto name = prefix + kActuators[i];
            auto &f = fingers_[i];
            f.actuator = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            if (f.actuator < 0) {
                throw std::invalid_argument("brainco_hand: no actuator named '" + name +
                                            "' (is the brainco_revo2 model attached with prefix '" + prefix + "'?)");
            }
            f.lo = mj_model_->actuator_ctrlrange[2 * f.actuator];
            f.hi = mj_model_->actuator_ctrlrange[2 * f.actuator + 1];
            f.fmax = std::max(std::abs(mj_model_->actuator_forcerange[2 * f.actuator]),
                              std::abs(mj_model_->actuator_forcerange[2 * f.actuator + 1]));
            f.vmax = vmax[i];
            f.q_adr = sensorAdr((name + "_pos").c_str());
            f.dq_adr = sensorAdr((name + "_vel").c_str());
            f.force_adr = sensorAdr((name + "_force").c_str());
            if (f.q_adr < 0 || f.dq_adr < 0 || f.force_adr < 0 || f.hi <= f.lo) {
                throw std::invalid_argument("brainco_hand: actuator '" + name + "' needs a ctrlrange and sensors " +
                                            name + "_pos, " + name + "_vel, " + name + "_force");
            }
            f.ctrl = f.lo;  // the hand service starts with q = 0 (open)
        }
        if (touch) {
            for (int i = 0; i < kFingers; i++) {
                auto name = prefix + kTouch[i] + "_touch";
                pads_[i].adr = sensorAdr(name.c_str());
                if (pads_[i].adr < 0) {
                    throw std::invalid_argument("brainco_hand: no touch sensor named '" + name + "' (or set touch: false)");
                }
                // optional: tangential force from <prefix><finger>_contact
                int id = mj_name2id(mj_model_, mjOBJ_SENSOR, (prefix + kTouch[i] + "_contact").c_str());
                if (id >= 0 && mj_model_->sensor_type[id] == mjSENS_CONTACT && mj_model_->sensor_dim[id] % 10 == 0) {
                    pads_[i].contact_adr = mj_model_->sensor_adr[id];
                    pads_[i].contact_num = mj_model_->sensor_dim[id] / 10;
                }
            }
            touch_ = std::make_unique<Touch_t>(topic + "/touch");
        }

        cmd_ = std::make_unique<Cmd_t>(topic + "/cmd");
        cmd_->msg_.cmds().resize(kMotors);
        for (auto &c : cmd_->msg_.cmds()) {
            c.dq() = 1.0;  // as brainco_hand_service: full speed until told otherwise
        }
        state_ = std::make_unique<State_t>(topic + "/state");
        state_->msg_.states().resize(kMotors);
    }

    void step() override
    {
        // Commands: move each position servo's setpoint towards the target at the commanded speed.
        double dt = std::clamp(mj_data_->time - last_time_, 0.0, 0.1);
        last_time_ = mj_data_->time;
        std::array<float, kMotors> q{}, dq{};
        {
            std::lock_guard<std::mutex> lock(cmd_->mutex_);
            const auto &cmds = cmd_->msg_.cmds();
            for (int i = 0; i < kMotors && i < static_cast<int>(cmds.size()); i++) {
                q[i] = std::clamp(cmds[i].q(), 0.f, 1.f);
                dq[i] = std::clamp(cmds[i].dq(), 0.f, 1.f);
            }
        }
        for (int i = 0; i < kMotors; i++) {
            auto &f = fingers_[i];
            double goal = f.lo + q[i] * (f.hi - f.lo);
            double step = dq[i] * f.vmax * dt;
            f.ctrl += std::clamp(goal - f.ctrl, -step, step);
            mj_data_->ctrl[f.actuator] = f.ctrl;
        }

        if ((tick_++ % div_) != 0) {
            return;
        }
        if (state_->trylock()) {
            for (int i = 0; i < kMotors; i++) {
                const auto &f = fingers_[i];
                auto &s = state_->msg_.states()[i];
                double qj = mj_data_->sensordata[f.q_adr];
                double vj = mj_data_->sensordata[f.dq_adr];
                s.q() = static_cast<float>(std::clamp((qj - f.lo) / (f.hi - f.lo), 0.0, 1.0));
                s.dq() = static_cast<float>(std::clamp(vj / f.vmax, -1.0, 1.0));
                s.tau_est() = static_cast<float>(std::clamp(mj_data_->sensordata[f.force_adr] / f.fmax, -1.0, 1.0));
                s.q_raw() = static_cast<float>(qj);
                s.dq_raw() = static_cast<float>(vj);
            }
            state_->unlockAndPublish();
        }
        if (touch_ && touch_->trylock()) {
            auto &p = touch_->msg_.pressure();
            for (int i = 0; i < kFingers; i++) {
                p[i] = static_cast<float>(mj_data_->sensordata[pads_[i].adr]);
                p[kFingers + i] = static_cast<float>(tangential(pads_[i]));
            }
            touch_->unlockAndPublish();
        }
    }

private:
    struct Finger
    {
        int actuator = -1, q_adr = -1, dq_adr = -1, force_adr = -1;
        double lo = 0, hi = 1, fmax = 1, vmax = 1, ctrl = 0;
    };
    struct Pad
    {
        int adr = -1, contact_adr = -1, contact_num = 0;
    };

    // Tangential force (N) on a pad: |sum of the tangential parts| of the contacts reported by the
    // pad's contact sensor (data="found force normal tangent": per contact 10 values, force in the
    // contact frame [normal, tangent, normal x tangent]). Computed in mj_step like the touch sensor.
    double tangential(const Pad &pad) const
    {
        if (pad.contact_adr < 0) {
            return 0;
        }
        double sum[3] = {0, 0, 0};
        for (int k = 0; k < pad.contact_num; k++) {
            const mjtNum *c = mj_data_->sensordata + pad.contact_adr + 10 * k;
            if (c[0] <= 0) {
                continue;
            }
            mjtNum t2[3];
            mju_cross(t2, c + 4, c + 7);
            for (int j = 0; j < 3; j++) {
                sum[j] += c[2] * c[7 + j] + c[3] * t2[j];
            }
        }
        return std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
    }

    std::array<Finger, kMotors> fingers_;
    std::array<Pad, kFingers> pads_;
    std::unique_ptr<Cmd_t> cmd_;
    std::unique_ptr<State_t> state_;
    std::unique_ptr<Touch_t> touch_;
    double last_time_ = 0;
    int div_ = 10;
    unsigned long tick_ = 0;
};
