#pragma once

#include <cmath>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/MotorCmd_.hpp>
#include <unitree/idl/hg/MotorState_.hpp>
#include <unitree/idl/ros2/Point32_.hpp>

#include "bridge_component.h"
#include "brainco_hand.h"
#include "dex3_hand.h"
#include "perception_components.h"

// rt/secondary_imu: torso IMU read from the secondary_imu_* sensors (G1).
class SecondaryImuComponent : public BridgeComponent
{
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;

public:
    SecondaryImuComponent(mjModel *model, mjData *data) : BridgeComponent(model, data)
    {
        quat_adr_ = sensorAdr("secondary_imu_quat");
        gyro_adr_ = sensorAdr("secondary_imu_gyro");
        acc_adr_ = sensorAdr("secondary_imu_acc");
        imustate_ = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void step() override
    {
        if (!imustate_->trylock()) {
            return;
        }
        auto &msg = imustate_->msg_;
        if (quat_adr_ >= 0) {
            for (int i = 0; i < 4; i++) {
                msg.quaternion()[i] = mj_data_->sensordata[quat_adr_ + i];
            }
            double w = msg.quaternion()[0];
            double x = msg.quaternion()[1];
            double y = msg.quaternion()[2];
            double z = msg.quaternion()[3];

            msg.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            msg.rpy()[1] = asin(2 * (w * y - z * x));
            msg.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        }
        if (gyro_adr_ >= 0) {
            for (int i = 0; i < 3; i++) {
                msg.gyroscope()[i] = mj_data_->sensordata[gyro_adr_ + i];
            }
        }
        if (acc_adr_ >= 0) {
            for (int i = 0; i < 3; i++) {
                msg.accelerometer()[i] = mj_data_->sensordata[acc_adr_ + i];
            }
        }
        imustate_->unlockAndPublish();
    }

private:
    int quat_adr_ = -1;
    int gyro_adr_ = -1;
    int acc_adr_ = -1;
    std::unique_ptr<IMUState_t> imustate_;
};

// rt/lf/bmsstate: battery state. Constant 100% charge in simulation.
class BmsStateComponent : public BridgeComponent
{
    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;

public:
    BmsStateComponent(mjModel *model, mjData *data) : BridgeComponent(model, data)
    {
        bmsstate_ = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate_->msg_.soc() = 100;
    }

    // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
    void step() override { bmsstate_->unlockAndPublish(); }

private:
    std::unique_ptr<BmsState_t> bmsstate_;
};

// rt/waistpitchcmd + rt/waistpitchstate: the G1-D waist pitch motor, which is not part of
// rt/lowcmd / rt/lowstate. Params: `actuator` (MJCF actuator name).
class WaistPitchComponent : public BridgeComponent
{
    using Cmd_t = unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::MotorCmd_>;
    using State_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::MotorState_>;

public:
    WaistPitchComponent(mjModel *model, mjData *data, const YAML::Node &params) : BridgeComponent(model, data)
    {
        auto name = params["actuator"].as<std::string>();
        actuator_ = mj_name2id(mj_model_, mjOBJ_ACTUATOR, name.c_str());
        if (actuator_ < 0) {
            throw std::invalid_argument("waist_pitch: no actuator named '" + name + "' in the model");
        }
        int joint = mj_model_->actuator_trnid[2 * actuator_];
        q_adr_ = sensorAdrByObj(mjSENS_JOINTPOS, mjOBJ_JOINT, joint);
        dq_adr_ = sensorAdrByObj(mjSENS_JOINTVEL, mjOBJ_JOINT, joint);
        tau_adr_ = sensorAdrByObj(mjSENS_JOINTACTFRC, mjOBJ_JOINT, joint);
        if (q_adr_ < 0 || dq_adr_ < 0 || tau_adr_ < 0) {
            throw std::invalid_argument("waist_pitch: joint of '" + name +
                                        "' needs jointpos, jointvel and jointactuatorfrc sensors");
        }
        cmd_ = std::make_unique<Cmd_t>("rt/waistpitchcmd");
        state_ = std::make_unique<State_t>("rt/waistpitchstate");
    }

    void step() override
    {
        {
            std::lock_guard<std::mutex> lock(cmd_->mutex_);
            auto &m = cmd_->msg_;
            mj_data_->ctrl[actuator_] = m.tau() + m.kp() * (m.q() - mj_data_->sensordata[q_adr_]) +
                                        m.kd() * (m.dq() - mj_data_->sensordata[dq_adr_]);
        }
        if (state_->trylock()) {
            state_->msg_.q() = mj_data_->sensordata[q_adr_];
            state_->msg_.dq() = mj_data_->sensordata[dq_adr_];
            state_->msg_.tau_est() = mj_data_->sensordata[tau_adr_];
            state_->unlockAndPublish();
        }
    }

private:
    int actuator_ = -1;
    int q_adr_ = -1;
    int dq_adr_ = -1;
    int tau_adr_ = -1;
    std::unique_ptr<Cmd_t> cmd_;
    std::unique_ptr<State_t> state_;
};

// rt/hispeed_state (geometry_msgs/Point32): y is the G1-D lifting column height in metres.
// Params: `sensor` (MJCF sensor that measures the column height, default "lift_pos").
class HispeedStateComponent : public BridgeComponent
{
    using State_t = unitree::robot::RealTimePublisher<geometry_msgs::msg::dds_::Point32_>;

public:
    HispeedStateComponent(mjModel *model, mjData *data, const YAML::Node &params) : BridgeComponent(model, data)
    {
        auto name = params["sensor"] ? params["sensor"].as<std::string>() : std::string("lift_pos");
        height_adr_ = sensorAdr(name.c_str());
        if (height_adr_ < 0) {
            throw std::invalid_argument("hispeed_state: no sensor named '" + name + "' in the model");
        }
        state_ = std::make_unique<State_t>("rt/hispeed_state");
    }

    void step() override
    {
        if (state_->trylock()) {
            state_->msg_.y() = static_cast<float>(mj_data_->sensordata[height_adr_]);
            state_->unlockAndPublish();
        }
    }

private:
    int height_adr_ = -1;
    std::unique_ptr<State_t> state_;
};

// rt/lf/lowstate: rt/lowstate at a lower rate. unitree_go::LowState_ or unitree_hg::LowState_
// depending on the robot's IDL.
class LfLowStateComponent : public LowRateComponent
{
public:
    LfLowStateComponent(mjModel *model, mjData *data, const YAML::Node &params, const BridgeContext &ctx)
        : LowRateComponent(model, data, params, "lf_lowstate"), ctx_(ctx)
    {
        if (ctx.go_lowstate) {
            go_ = std::make_unique<unitree::robot::go2::publisher::LowState>("rt/lf/lowstate");
            go_->joystick = ctx.joystick;
        } else {
            hg_ = std::make_unique<unitree::robot::g1::publisher::LowState>("rt/lf/lowstate");
            hg_->joystick = ctx.joystick;
        }
    }

    void step() override
    {
        if (!due()) {
            return;
        }
        if (go_) {
            if (go_->trylock()) {
                go_->msg_ = *ctx_.go_lowstate;
                go_->unlockAndPublish();
            }
        } else if (hg_->trylock()) {
            hg_->msg_ = *ctx_.hg_lowstate;
            hg_->unlockAndPublish();
        }
    }

private:
    BridgeContext ctx_;
    std::unique_ptr<unitree::robot::go2::publisher::LowState> go_;
    std::unique_ptr<unitree::robot::g1::publisher::LowState> hg_;
};

// rt/lf/sportmodestate (unitree_go::SportModeState_): rt/sportmodestate at a lower rate.
class LfSportModeStateComponent : public LowRateComponent
{
public:
    LfSportModeStateComponent(mjModel *model, mjData *data, const YAML::Node &params, const BridgeContext &ctx)
        : LowRateComponent(model, data, params, "lf_sportmodestate"), ctx_(ctx),
          pub_(std::make_unique<unitree::robot::go2::publisher::SportModeState>("rt/lf/sportmodestate"))
    {
    }

    void step() override
    {
        if (due() && pub_->trylock()) {
            pub_->msg_ = *ctx_.sportmodestate;
            pub_->unlockAndPublish();
        }
    }

private:
    BridgeContext ctx_;
    std::unique_ptr<unitree::robot::go2::publisher::SportModeState> pub_;
};

inline void RegisterBuiltinComponents()
{
    auto &registry = ComponentRegistry::instance();
    registry.add("secondary_imu", [](mjModel *m, mjData *d, const YAML::Node &, const BridgeContext &) { return std::make_unique<SecondaryImuComponent>(m, d); });
    registry.add("bms_state", [](mjModel *m, mjData *d, const YAML::Node &, const BridgeContext &) { return std::make_unique<BmsStateComponent>(m, d); });
    registry.add("waist_pitch", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<WaistPitchComponent>(m, d, p); });
    registry.add("hispeed_state", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<HispeedStateComponent>(m, d, p); });
    registry.add("lf_lowstate", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &c) { return std::make_unique<LfLowStateComponent>(m, d, p, c); });
    registry.add("brainco_hand", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<BraincoHandComponent>(m, d, p); });
    registry.add("dex3_hand", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<Dex3HandComponent>(m, d, p); });
    registry.add("lf_sportmodestate", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &c) { return std::make_unique<LfSportModeStateComponent>(m, d, p, c); });
    RegisterPerceptionComponents();
}
