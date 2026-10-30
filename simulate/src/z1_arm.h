#pragma once

// Unitree Z1 arm (6 joints + Unitree gripper) as a DDS low-level motor interface.
//
// The real Z1 has no DDS interface: its controller (z1_controller / z1_ros) talks UDP to the arm
// and to the z1_sdk, and its ROS wrappers are ROS 1. The topics below are therefore defined by
// this simulator (see readme.md, "B2 with Z1 arm"), with message types unitree_sdk2 ships:
//
//   rt/z1/cmd    unitree_go::msg::dds_::MotorCmds_    (7 cmds)
//   rt/z1/state  unitree_go::msg::dds_::MotorStates_  (7 states, 500 Hz)
//
// Motor order: 0..5 = joint1..joint6, 6 = gripper (z1_sdk LowlevelCmd / LowlevelState order).
// Command, per motor, with the z1_ros low-level gains (SI units, no 25.6 / 0.0128 scaling):
//
//   tau_motor = tau + kp * (q - q_meas) + kd * (dq - dq_meas)       (N m, rad, rad/s)
//
// when cmd.mode != 0; cmd.mode == 0 (also before the first command) leaves the motor unpowered
// (zero torque, the arm rests on its joint limits / the model's joint damping and friction). No
// gravity compensation is added: as with z1_sdk LOWCMD, the client sends it in `tau`. The torque is
// clamped to the joint's effort limit (actuator ctrlrange: 30 N m, joint2 60 N m). Fields
// reserve and the command timeout are not used; there is no watchdog (the last command is held).
//
// State: mode = 1 when the PD law is active else 0, q, dq, ddq (joint acceleration), tau_est (joint
// actuator torque); q_raw / dq_raw / ddq_raw, temperature, lost, reserve = 0.
//
// The arm model is unitree_robots/components/z1/z1.xml attached with a prefix (see robot_compose.h);
// this component finds its elements by prefixed name: actuators <prefix>{joint1..joint6, gripper},
// sensors <prefix><actuator>_{pos,vel,torque}.
//
// YAML params: prefix (default "z1_"), topic (default "rt/z1"), hz (state rate, default 500).

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

#include <algorithm>
#include <array>
#include <cmath>

#include "bridge_component.h"

class Z1ArmComponent : public BridgeComponent
{
    using Cmd_t = unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>;
    using State_t = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>;

    static constexpr int kMotors = 7;
    static constexpr double kBridgeHz = 1000.0;

public:
    Z1ArmComponent(mjModel *model, mjData *data, const YAML::Node &params) : BridgeComponent(model, data)
    {
        static const char *kNames[kMotors] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};

        auto prefix = params["prefix"] ? params["prefix"].as<std::string>() : std::string("z1_");
        auto topic = params["topic"] ? params["topic"].as<std::string>() : std::string("rt/z1");
        if (params["hz"]) {
            div_ = static_cast<int>(std::lround(kBridgeHz / params["hz"].as<double>()));
        }
        if (div_ < 1) {
            throw std::invalid_argument("z1_arm: hz must be <= 1000");
        }

        for (int i = 0; i < kMotors; i++) {
            auto name = prefix + kNames[i];
            auto &m = motors_[i];
            m.actuator = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
            if (m.actuator < 0) {
                throw std::invalid_argument("z1_arm: no actuator named '" + name +
                                            "' (is the z1 model attached with prefix '" + prefix + "'?)");
            }
            m.q_adr = sensorAdr((name + "_pos").c_str());
            m.dq_adr = sensorAdr((name + "_vel").c_str());
            m.tau_adr = sensorAdr((name + "_torque").c_str());
            if (m.q_adr < 0 || m.dq_adr < 0 || m.tau_adr < 0) {
                throw std::invalid_argument("z1_arm: actuator '" + name + "' needs sensors " + name + "_pos, " +
                                            name + "_vel, " + name + "_torque");
            }
            m.dof = mj_model_->jnt_dofadr[mj_model_->actuator_trnid[2 * m.actuator]];
        }

        cmd_ = std::make_unique<Cmd_t>(topic + "/cmd");
        cmd_->msg_.cmds().resize(kMotors);
        state_ = std::make_unique<State_t>(topic + "/state");
        state_->msg_.states().resize(kMotors);
    }

    void step() override
    {
        {
            std::lock_guard<std::mutex> lock(cmd_->mutex_);
            const auto &cmds = cmd_->msg_.cmds();
            for (int i = 0; i < kMotors; i++) {
                auto &m = motors_[i];
                m.enabled = false;
                double tau = 0;
                if (i < static_cast<int>(cmds.size()) && cmds[i].mode() != 0) {
                    const auto &c = cmds[i];
                    m.enabled = true;
                    tau = c.tau() + c.kp() * (c.q() - mj_data_->sensordata[m.q_adr]) +
                          c.kd() * (c.dq() - mj_data_->sensordata[m.dq_adr]);
                }
                const mjtNum *r = mj_model_->actuator_ctrlrange + 2 * m.actuator;
                mj_data_->ctrl[m.actuator] = std::isfinite(tau) ? std::clamp(tau, r[0], r[1]) : 0.0;
            }
        }

        if ((tick_++ % div_) == 0 && state_->trylock()) {
            const mjtNum *sd = mj_data_->sensordata;
            for (int i = 0; i < kMotors; i++) {
                const auto &m = motors_[i];
                auto &s = state_->msg_.states()[i];
                s.mode() = m.enabled ? 1 : 0;
                s.q() = static_cast<float>(sd[m.q_adr]);
                s.dq() = static_cast<float>(sd[m.dq_adr]);
                s.ddq() = static_cast<float>(mj_data_->qacc[m.dof]);
                s.tau_est() = static_cast<float>(sd[m.tau_adr]);
            }
            state_->unlockAndPublish();
        }
    }

private:
    struct Motor
    {
        int actuator = -1, q_adr = -1, dq_adr = -1, tau_adr = -1, dof = -1;
        bool enabled = false;
    };

    std::array<Motor, kMotors> motors_;
    std::unique_ptr<Cmd_t> cmd_;
    std::unique_ptr<State_t> state_;
    int div_ = 2;  // 500 Hz
    unsigned long tick_ = 0;
};
