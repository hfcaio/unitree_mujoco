#pragma once

// Unitree Dex3-1 three-finger hand, with the topics the real G1 uses (unitree_sdk2
// example/g1/dex3/g1_dex3_example.cpp, unitree_ros2 example/src/src/g1/dex3), one set per hand:
//
//   rt/dex3/<side>/cmd          unitree_hg::msg::dds_::HandCmd_    (7 motor_cmd)
//   rt/dex3/<side>/state        unitree_hg::msg::dds_::HandState_  (7 motor_state, 9 press_sensor_state, imu_state)
//   rt/lf/dex3/<side>/state     the same message at a lower rate (the SDK example reads it)
//
// Motor order (both hands): 0 thumb_0, 1 thumb_1, 2 thumb_2, 3 middle_0, 4 middle_1, 5 index_0, 6 index_1.
// Command, per motor, as the real hand evaluates it:
//
//   tau_motor = tau + kp * (q - q_meas) + kd * (dq - dq_meas)
//
// `mode` packs RIS_Mode_t of the example: bits 0-3 motor id, bits 4-6 status, bit 7 timeout. A
// status of 0 (also the state before the first command) leaves the motor unpowered (zero torque);
// any other status enables the PD law. The id and timeout bits are not used; no CRC is checked
// (HandCmd_ has none). The result is clamped to the joint's effort limit (the actuator's ctrlrange).
//
// State: q, dq, tau_est (joint actuator torque) from the joint's sensors, mode = id | enabled << 4,
// ddq / temperature / vol / sensor / motorstate = 0; imu_state from the palm IMU sensors of the model;
// press_sensor_state[i].pressure[0] = normal force (N) of the touch sensor hand_press_<i> (the other
// 11 pressure and all 12 temperature entries stay 0; lost = 0); power_v = system_v = device_v = 24,
// power_a = 0.1 (constants).
//
// The hand model is unitree_robots/components/dex3_1/<side>.xml attached with a prefix (see
// robot_compose.h); this component finds its elements by prefixed name: actuators
// <prefix>hand_<motor>, sensors <prefix>hand_<motor>_{pos,vel,torque}, touch sensors
// <prefix>hand_press_<0..8> and optionally <prefix>hand_imu_{quat,gyro,acc}. The touch values are
// computed inside mj_step; the bridge thread only reads sensordata.
//
// YAML params: side (left|right, required), prefix (default "<side>_"), topic (default
// "rt/dex3/<side>"), hz (state rate, default 100), lf_hz (rt/lf/... rate, default 10; 0 = off).

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include "bridge_component.h"

class Dex3HandComponent : public BridgeComponent
{
    using Cmd_t = unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::HandCmd_>;
    using State_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::HandState_>;

    static constexpr int kMotors = 7;
    static constexpr int kSensors = 9;
    static constexpr double kBridgeHz = 1000.0;

public:
    Dex3HandComponent(mjModel *model, mjData *data, const YAML::Node &params) : BridgeComponent(model, data)
    {
        static const char *kNames[kMotors] = {"thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"};

        if (!params["side"]) {
            throw std::invalid_argument("dex3_hand: param `side` (left or right) is required");
        }
        auto side = params["side"].as<std::string>();
        auto prefix = params["prefix"] ? params["prefix"].as<std::string>() : side + "_";
        auto topic = params["topic"] ? params["topic"].as<std::string>() : "rt/dex3/" + side;
        if (params["hz"]) {
            div_ = static_cast<int>(std::lround(kBridgeHz / params["hz"].as<double>()));
        }
        double lf_hz = params["lf_hz"] ? params["lf_hz"].as<double>() : 10.0;
        if (div_ < 1 || lf_hz > kBridgeHz || lf_hz < 0) {
            throw std::invalid_argument("dex3_hand: hz and lf_hz must be <= 1000");
        }
        lf_div_ = lf_hz > 0 ? static_cast<int>(std::lround(kBridgeHz / lf_hz)) : 0;

        for (int i = 0; i < kMotors; i++) {
            auto name = prefix + "hand_" + kNames[i];
            auto &m = motors_[i];
            m.actuator = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            if (m.actuator < 0) {
                throw std::invalid_argument("dex3_hand: no actuator named '" + name +
                                            "' (is the dex3_1 model attached with prefix '" + prefix + "'?)");
            }
            m.q_adr = sensorAdr((name + "_pos").c_str());
            m.dq_adr = sensorAdr((name + "_vel").c_str());
            m.tau_adr = sensorAdr((name + "_torque").c_str());
            if (m.q_adr < 0 || m.dq_adr < 0 || m.tau_adr < 0) {
                throw std::invalid_argument("dex3_hand: actuator '" + name + "' needs sensors " + name + "_pos, " +
                                            name + "_vel, " + name + "_torque");
            }
        }
        for (int i = 0; i < kSensors; i++) {
            auto name = prefix + "hand_press_" + std::to_string(i);
            press_adr_[i] = sensorAdr(name.c_str());
            if (press_adr_[i] < 0) {
                throw std::invalid_argument("dex3_hand: no touch sensor named '" + name + "'");
            }
        }
        // optional palm IMU
        imu_quat_ = sensorAdr((prefix + "hand_imu_quat").c_str());
        imu_gyro_ = sensorAdr((prefix + "hand_imu_gyro").c_str());
        imu_acc_ = sensorAdr((prefix + "hand_imu_acc").c_str());

        cmd_ = std::make_unique<Cmd_t>(topic + "/cmd");
        cmd_->msg_.motor_cmd().resize(kMotors);
        state_ = std::make_unique<State_t>(topic + "/state");
        InitState(state_->msg_);
        if (lf_div_ > 0) {
            lf_state_ = std::make_unique<State_t>("rt/lf/dex3/" + side + "/state");
            InitState(lf_state_->msg_);
        }
    }

    void step() override
    {
        {
            std::lock_guard<std::mutex> lock(cmd_->mutex_);
            const auto &cmds = cmd_->msg_.motor_cmd();
            for (int i = 0; i < kMotors; i++) {
                auto &m = motors_[i];
                m.enabled = false;
                double tau = 0;
                if (i < static_cast<int>(cmds.size()) && ((cmds[i].mode() >> 4) & 0x7) != 0) {
                    const auto &c = cmds[i];
                    m.enabled = true;
                    tau = c.tau() + c.kp() * (c.q() - mj_data_->sensordata[m.q_adr]) +
                          c.kd() * (c.dq() - mj_data_->sensordata[m.dq_adr]);
                }
                const mjtNum *r = mj_model_->actuator_ctrlrange + 2 * m.actuator;
                mj_data_->ctrl[m.actuator] = std::isfinite(tau) ? std::clamp(tau, r[0], r[1]) : 0.0;
            }
        }

        bool main_due = (tick_ % div_) == 0;
        bool lf_due = lf_div_ > 0 && (tick_ % lf_div_) == 0;
        tick_++;
        if (main_due && state_->trylock()) {
            Fill(state_->msg_);
            // copy for the low-frequency topic if it falls on this tick too
            if (lf_due && lf_state_->trylock()) {
                lf_state_->msg_ = state_->msg_;
                lf_state_->unlockAndPublish();
                lf_due = false;
            }
            state_->unlockAndPublish();
        }
        if (lf_due && lf_state_->trylock()) {
            Fill(lf_state_->msg_);
            lf_state_->unlockAndPublish();
        }
    }

private:
    struct Motor
    {
        int actuator = -1, q_adr = -1, dq_adr = -1, tau_adr = -1;
        bool enabled = false;
    };

    static void InitState(unitree_hg::msg::dds_::HandState_ &s)
    {
        s.motor_state().resize(kMotors);
        s.press_sensor_state().resize(kSensors);
        s.power_v() = 24.0f;
        s.power_a() = 0.1f;
        s.system_v() = 24.0f;
        s.device_v() = 24.0f;
    }

    void Fill(unitree_hg::msg::dds_::HandState_ &s) const
    {
        const mjtNum *sd = mj_data_->sensordata;
        for (int i = 0; i < kMotors; i++) {
            const auto &m = motors_[i];
            auto &ms = s.motor_state()[i];
            ms.mode() = static_cast<uint8_t>((i & 0xF) | ((m.enabled ? 1 : 0) << 4));
            ms.q() = static_cast<float>(sd[m.q_adr]);
            ms.dq() = static_cast<float>(sd[m.dq_adr]);
            ms.tau_est() = static_cast<float>(sd[m.tau_adr]);
        }
        for (int i = 0; i < kSensors; i++) {
            s.press_sensor_state()[i].pressure()[0] = static_cast<float>(sd[press_adr_[i]]);
        }
        auto &imu = s.imu_state();
        if (imu_quat_ >= 0) {
            for (int i = 0; i < 4; i++) {
                imu.quaternion()[i] = static_cast<float>(sd[imu_quat_ + i]);
            }
            double w = sd[imu_quat_], x = sd[imu_quat_ + 1], y = sd[imu_quat_ + 2], z = sd[imu_quat_ + 3];
            imu.rpy()[0] = static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
            imu.rpy()[1] = static_cast<float>(std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)));
            imu.rpy()[2] = static_cast<float>(std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));
        }
        for (int i = 0; i < 3; i++) {
            if (imu_gyro_ >= 0) {
                imu.gyroscope()[i] = static_cast<float>(sd[imu_gyro_ + i]);
            }
            if (imu_acc_ >= 0) {
                imu.accelerometer()[i] = static_cast<float>(sd[imu_acc_ + i]);
            }
        }
    }

    std::array<Motor, kMotors> motors_;
    std::array<int, kSensors> press_adr_{};
    int imu_quat_ = -1, imu_gyro_ = -1, imu_acc_ = -1;
    std::unique_ptr<Cmd_t> cmd_;
    std::unique_ptr<State_t> state_, lf_state_;
    int div_ = 10, lf_div_ = 100;
    unsigned long tick_ = 0;
};
