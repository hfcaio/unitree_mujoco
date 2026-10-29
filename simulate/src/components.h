#pragma once

#include <cmath>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include "bridge_component.h"

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

inline void RegisterBuiltinComponents()
{
    auto &registry = ComponentRegistry::instance();
    registry.add("secondary_imu", [](mjModel *m, mjData *d) { return std::make_unique<SecondaryImuComponent>(m, d); });
    registry.add("bms_state", [](mjModel *m, mjData *d) { return std::make_unique<BmsStateComponent>(m, d); });
}
