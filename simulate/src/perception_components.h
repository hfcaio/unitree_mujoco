#pragma once

// Perception / state-estimation topics of a real Go2 that are not motor state (the rt/utlidar/*
// family published next to rt/lowstate on the robot). See readme.md "Perception topics" for which
// of the real robot's topics are simulated, where their types come from and what is guessed.
//
// Threading: BridgeComponent::step() runs on the 1 kHz bridge thread, which is not the physics
// thread. Topics that only need the base pose / IMU read sensordata like rt/lowstate and
// rt/sportmodestate do. Anything that needs body poses (foot positions) or ray casting works on a
// private mjData (KinematicsSnapshot): the generalized state is copied from the shared mjData while
// holding the physics mutex (mj_step runs under it), and mj_kinematics / mj_multiRay then run on
// the private copy with no lock held, so the shared mjData (and its stack) is never written to and
// the physics thread only waits for a memcpy of qpos. The lidar does its ray casting in its own
// thread, so neither the physics thread nor the bridge tick pays for it.

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/idl/go2/LidarState_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/Odometry_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/PoseStamped_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bridge_component.h"

namespace perception
{

// Stamps use the wall clock, as the real robot does (the simulator publishes no /clock).
inline builtin_interfaces::msg::dds_::Time_ WallStamp()
{
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch()).count();
    builtin_interfaces::msg::dds_::Time_ t;
    t.sec(static_cast<int32_t>(ns / 1000000000LL));
    t.nanosec(static_cast<uint32_t>(ns % 1000000000LL));
    return t;
}

inline double WallSeconds()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

inline void SetHeader(std_msgs::msg::dds_::Header_ &h, const std::string &frame_id)
{
    h.stamp(WallStamp());
    h.frame_id(frame_id);
}

inline void SetQuat(geometry_msgs::msg::dds_::Quaternion_ &q, const mjtNum wxyz[4])
{
    q.w(wxyz[0]);
    q.x(wxyz[1]);
    q.y(wxyz[2]);
    q.z(wxyz[3]);
}

inline void SetVec(geometry_msgs::msg::dds_::Vector3_ &v, const mjtNum xyz[3])
{
    v.x(xyz[0]);
    v.y(xyz[1]);
    v.z(xyz[2]);
}

inline void QuatToRpy(const mjtNum q[4], float rpy[3])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    rpy[0] = static_cast<float>(std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)));
    rpy[1] = static_cast<float>(std::asin(std::fmax(-1.0, std::fmin(1.0, 2 * (w * y - z * x)))));
    rpy[2] = static_cast<float>(std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)));
}

// sensor_msgs/PointField datatypes
enum : uint8_t { kUint16 = 4, kFloat32 = 7 };

inline sensor_msgs::msg::dds_::PointField_ Field(const char *name, uint32_t offset, uint8_t datatype)
{
    sensor_msgs::msg::dds_::PointField_ f;
    f.name(name);
    f.offset(offset);
    f.datatype(datatype);
    f.count(1);
    return f;
}

template <typename T>
T Param(const YAML::Node &params, const char *key, const T &fallback)
{
    return params && params[key] ? params[key].as<T>() : fallback;
}

// A private mjData that mirrors the simulation's kinematic state (see the threading note above).
class KinematicsSnapshot
{
public:
    explicit KinematicsSnapshot(const mjModel *model) : m_(model), d_(mj_makeData(model)) {}
    ~KinematicsSnapshot() { mj_deleteData(d_); }
    KinematicsSnapshot(const KinematicsSnapshot &) = delete;
    KinematicsSnapshot &operator=(const KinematicsSnapshot &) = delete;

    // Copies qpos / mocap / time from `src` under `mtx` and runs forward kinematics on the copy.
    // blocking = false only tries the lock and returns false when the physics thread holds it.
    // Without a mutex (no physics thread to coordinate with) the copy is taken unlocked.
    bool refresh(const mjData *src, std::recursive_mutex *mtx, bool blocking)
    {
        std::unique_lock<std::recursive_mutex> lock;
        if (mtx) {
            lock = std::unique_lock<std::recursive_mutex>(*mtx, std::defer_lock);
            if (blocking) {
                lock.lock();
            } else if (!lock.try_lock()) {
                return false;
            }
        }
        d_->time = src->time;
        mju_copy(d_->qpos, src->qpos, m_->nq);
        mju_copy(d_->mocap_pos, src->mocap_pos, 3 * m_->nmocap);
        mju_copy(d_->mocap_quat, src->mocap_quat, 4 * m_->nmocap);
        if (lock.owns_lock()) {
            lock.unlock();
        }
        mj_kinematics(m_, d_);
        return true;
    }

    mjData *data() const { return d_; }

private:
    const mjModel *m_;
    mjData *d_;
};

// Where a sensor sits: a site (`site: name`) or a body plus a fixed offset
// (`body: name`, `pos: [x, y, z]`, `rpy: [roll, pitch, yaw]` in radians, URDF convention).
struct Mount
{
    int body = -1;
    mjtNum pos[3] = {0, 0, 0};
    mjtNum quat[4] = {1, 0, 0, 0};

    static Mount Parse(const mjModel *m, const YAML::Node &params, const std::string &component,
                       const std::string &default_site)
    {
        Mount mount;
        if (params && params["body"]) {
            auto name = params["body"].as<std::string>();
            mount.body = mj_name2id(m, mjOBJ_BODY, name.c_str());
            if (mount.body < 0) {
                throw std::invalid_argument(component + ": no body named '" + name + "' in the model");
            }
            auto pos = Param(params, "pos", std::vector<double>{0, 0, 0});
            auto rpy = Param(params, "rpy", std::vector<double>{0, 0, 0});
            if (pos.size() != 3 || rpy.size() != 3) {
                throw std::invalid_argument(component + ": pos and rpy need 3 values");
            }
            for (int i = 0; i < 3; i++) {
                mount.pos[i] = pos[i];
            }
            // URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
            mjtNum qx[4], qy[4], qz[4], qzy[4];
            mjtNum ax[3] = {1, 0, 0}, ay[3] = {0, 1, 0}, az[3] = {0, 0, 1};
            mju_axisAngle2Quat(qx, ax, rpy[0]);
            mju_axisAngle2Quat(qy, ay, rpy[1]);
            mju_axisAngle2Quat(qz, az, rpy[2]);
            mju_mulQuat(qzy, qz, qy);
            mju_mulQuat(mount.quat, qzy, qx);
        } else {
            auto name = Param<std::string>(params, "site", default_site);
            int site = mj_name2id(m, mjOBJ_SITE, name.c_str());
            if (site < 0) {
                throw std::invalid_argument(component + ": no site named '" + name +
                                            "' in the model (set `site`, or `body` + `pos` + `rpy`)");
            }
            mount.body = m->site_bodyid[site];
            mju_copy3(mount.pos, m->site_pos + 3 * site);
            mju_copy4(mount.quat, m->site_quat + 4 * site);
        }
        return mount;
    }

    // World pose of the mount from a kinematics-filled mjData.
    void worldPose(const mjData *d, mjtNum p[3], mjtNum q[4]) const
    {
        mju_rotVecQuat(p, pos, d->xquat + 4 * body);
        mju_addTo3(p, d->xpos + 3 * body);
        mju_mulQuat(q, d->xquat + 4 * body, quat);
    }
};

// Base pose and velocity from the IMU-site sensors the base bridge uses for rt/lowstate and
// rt/sportmodestate (imu_quat, imu_gyro, imu_acc, frame_pos, frame_vel), moved from the IMU
// site to the frame of the body that carries it (base_link).
class BaseState
{
public:
    BaseState(const mjModel *m, const std::string &component)
    {
        auto adr = [&](const char *name) {
            int id = mj_name2id(m, mjOBJ_SENSOR, name);
            if (id < 0) {
                throw std::invalid_argument(component + ": the model needs a sensor named '" + name + "'");
            }
            return id;
        };
        int gyro = adr("imu_gyro");
        quat_ = m->sensor_adr[adr("imu_quat")];
        gyro_ = m->sensor_adr[gyro];
        acc_ = m->sensor_adr[adr("imu_acc")];
        site_ = m->sensor_objid[gyro];  // gyro sensors are always attached to a site
        body_ = m->site_bodyid[site_];
        mju_copy3(site_pos_, m->site_pos + 3 * site_);
        mju_copy4(site_quat_, m->site_quat + 4 * site_);
        int pos = mj_name2id(m, mjOBJ_SENSOR, "frame_pos");
        int vel = mj_name2id(m, mjOBJ_SENSOR, "frame_vel");
        pos_ = pos >= 0 ? m->sensor_adr[pos] : -1;
        vel_ = vel >= 0 ? m->sensor_adr[vel] : -1;
    }

    int site() const { return site_; }
    int body() const { return body_; }

    // IMU site orientation (world), angular velocity and specific force (site frame).
    void imu(const mjData *d, mjtNum q[4], mjtNum gyro[3], mjtNum acc[3]) const
    {
        mju_copy4(q, d->sensordata + quat_);
        mju_normalize4(q);
        mju_copy3(gyro, d->sensordata + gyro_);
        mju_copy3(acc, d->sensordata + acc_);
    }

    // Body-frame origin pose (world), linear velocity (world) and angular velocity (world).
    void bodyState(const mjData *d, mjtNum p[3], mjtNum q[4], mjtNum v[3], mjtNum w[3]) const
    {
        mjtNum qs[4], gyro[3], acc[3], qsinv[4], r[3], wxr[3];
        imu(d, qs, gyro, acc);
        mju_negQuat(qsinv, site_quat_);
        mju_mulQuat(q, qs, qsinv);
        mju_rotVecQuat(r, site_pos_, q);  // site offset in world
        mju_rotVecQuat(w, gyro, qs);
        if (pos_ >= 0) {
            mju_sub3(p, d->sensordata + pos_, r);
        } else {
            mju_zero3(p);
        }
        if (vel_ >= 0) {
            mju_cross(wxr, w, r);
            mju_sub3(v, d->sensordata + vel_, wxr);
        } else {
            mju_zero3(v);
        }
    }

private:
    int quat_ = -1, gyro_ = -1, acc_ = -1, pos_ = -1, vel_ = -1;
    int site_ = -1, body_ = -1;
    mjtNum site_pos_[3] = {0, 0, 0};
    mjtNum site_quat_[4] = {1, 0, 0, 0};
};

// What the utlidar_* components share so utlidar_lidar_state can report the cloud and IMU.
struct UtlidarStats
{
    std::mutex mtx;
    float cloud_frequency = 0;
    uint32_t cloud_size = 0;
    uint32_t cloud_scan_num = 0;
    float imu_frequency = 0;
    float imu_rpy[3] = {0, 0, 0};

    static UtlidarStats &instance()
    {
        static UtlidarStats stats;
        return stats;
    }
};

// Measured publish rate (exponential average of the interval between calls).
class RateMeter
{
public:
    float tick()
    {
        auto now = std::chrono::steady_clock::now();
        if (have_last_) {
            double dt = std::chrono::duration<double>(now - last_).count();
            avg_dt_ = avg_dt_ > 0 ? 0.9 * avg_dt_ + 0.1 * dt : dt;
        }
        last_ = now;
        have_last_ = true;
        return avg_dt_ > 0 ? static_cast<float>(1.0 / avg_dt_) : 0.f;
    }

private:
    std::chrono::steady_clock::time_point last_;
    bool have_last_ = false;
    double avg_dt_ = 0;
};

} // namespace perception

// rt/utlidar/cloud (sensor_msgs/PointCloud2), the Go2 L1 lidar: rays cast against the MuJoCo scene.
// Point layout as on the robot: x, y, z, intensity (float32 at 0, 4, 8, 16), ring (uint16 at 20),
// time (float32 at 24, seconds since the scan start), point_step 32, height 1, is_dense true, only
// hits are sent, in the sensor frame `frame_id`.
// The scan pattern is non-repetitive like the L1's: each scan casts `points` rays along a 2-D
// low-discrepancy sequence over the field of view (uniform per solid angle) that continues from
// scan to scan, so the union of consecutive scans fills the field of view.
// Params (all optional): site (default `utlidar`) or body + pos + rpy; hz (15); points (1440);
// h_fov ([-180, 180] deg azimuth around the sensor z axis); v_fov ([0, 90] deg elevation above the
// sensor xy plane); range ([0.05, 30] m); rings (18, elevation bins reported in `ring`);
// noise (0, std. dev. of the range in m); geomgroups ([0, 1, 2], geom groups the rays hit);
// exclude_body (default: the mount body; `none` to hit it too); topic (rt/utlidar/cloud);
// frame_id (utlidar_lidar); deskewed_topic (rt/utlidar/cloud_deskewed, `""` = off): the same
// points in the world frame `deskewed_frame_id` (odom).
class UtlidarCloudComponent : public BridgeComponent
{
    using Cloud_t = sensor_msgs::msg::dds_::PointCloud2_;
    using Pub_t = unitree::robot::PublisherBase<Cloud_t>;

public:
    UtlidarCloudComponent(mjModel *model, mjData *data, const YAML::Node &params, const BridgeContext &ctx)
        : BridgeComponent(model, data), sim_mutex_(ctx.sim_mutex), snapshot_(model)
    {
        using perception::Param;
        const std::string name = "utlidar_cloud";
        mount_ = perception::Mount::Parse(model, params, name, "utlidar");
        hz_ = Param(params, "hz", 15.0);
        npoints_ = Param(params, "points", 1440);
        auto h = Param(params, "h_fov", std::vector<double>{-180, 180});
        auto v = Param(params, "v_fov", std::vector<double>{0, 90});
        auto range = Param(params, "range", std::vector<double>{0.05, 30});
        rings_ = Param(params, "rings", 18);
        noise_ = Param(params, "noise", 0.0);
        auto groups = Param(params, "geomgroups", std::vector<int>{0, 1, 2});
        frame_id_ = Param<std::string>(params, "frame_id", "utlidar_lidar");
        deskewed_frame_id_ = Param<std::string>(params, "deskewed_frame_id", "odom");
        auto topic = Param<std::string>(params, "topic", "rt/utlidar/cloud");
        auto deskewed_topic = Param<std::string>(params, "deskewed_topic", "rt/utlidar/cloud_deskewed");
        if (hz_ <= 0 || npoints_ < 1 || rings_ < 1 || h.size() != 2 || v.size() != 2 || range.size() != 2 ||
            h[1] <= h[0] || v[1] <= v[0] || v[0] < -90 || v[1] > 90 || range[1] <= range[0] || noise_ < 0) {
            throw std::invalid_argument(name + ": invalid hz / points / rings / h_fov / v_fov / range / noise");
        }
        az0_ = h[0] * M_PI / 180;
        az_span_ = (h[1] - h[0]) * M_PI / 180;
        sin_el0_ = std::sin(v[0] * M_PI / 180);
        sin_el_span_ = std::sin(v[1] * M_PI / 180) - sin_el0_;
        el0_ = v[0] * M_PI / 180;
        el_span_ = (v[1] - v[0]) * M_PI / 180;
        range_min_ = range[0];
        range_max_ = range[1];
        for (int g : groups) {
            if (g < 0 || g >= mjNGROUP) {
                throw std::invalid_argument(name + ": geomgroups must be in [0, " + std::to_string(mjNGROUP - 1) + "]");
            }
            geomgroup_[g] = 1;
        }
        exclude_body_ = mount_.body;
        if (params && params["exclude_body"]) {
            auto ex = params["exclude_body"].as<std::string>();
            exclude_body_ = ex == "none" ? -1 : mj_name2id(model, mjOBJ_BODY, ex.c_str());
            if (ex != "none" && exclude_body_ < 0) {
                throw std::invalid_argument(name + ": no body named '" + ex + "' in the model");
            }
        }
        if (!sim_mutex_) {
            std::cerr << name << ": no physics mutex, the state is copied without locking" << std::endl;
        }

        dir_.resize(3 * npoints_);
        vec_.resize(3 * npoints_);
        ring_.resize(npoints_);
        geomid_.resize(npoints_);
        dist_.resize(npoints_);

        pub_ = std::make_unique<Pub_t>(topic);
        InitCloud(cloud_, frame_id_);
        if (!deskewed_topic.empty()) {
            deskewed_pub_ = std::make_unique<Pub_t>(deskewed_topic);
            InitCloud(deskewed_, deskewed_frame_id_);
        }
        thread_ = std::thread([this]() { this->loop(); });
    }

    ~UtlidarCloudComponent() override
    {
        {
            std::lock_guard<std::mutex> lock(stop_mtx_);
            stop_ = true;
        }
        stop_cv_.notify_all();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    // The scan runs in its own thread at `hz`; nothing to do on the bridge tick.
    void step() override {}

private:
    static constexpr uint32_t kPointStep = 32;

    static void InitCloud(Cloud_t &c, const std::string &frame_id)
    {
        using perception::Field;
        c.header().frame_id(frame_id);
        c.height(1);
        c.fields({Field("x", 0, perception::kFloat32), Field("y", 4, perception::kFloat32),
                  Field("z", 8, perception::kFloat32), Field("intensity", 16, perception::kFloat32),
                  Field("ring", 20, perception::kUint16), Field("time", 24, perception::kFloat32)});
        c.is_bigendian(false);
        c.point_step(kPointStep);
        c.is_dense(true);
    }

    void loop()
    {
        auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / hz_));
        auto next = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lock(stop_mtx_);
        while (!stop_) {
            lock.unlock();
            scan();
            lock.lock();
            next += period;
            auto now = std::chrono::steady_clock::now();
            if (next < now) {
                next = now;  // fell behind (e.g. too many rays): do not try to catch up
            }
            stop_cv_.wait_until(lock, next, [this]() { return stop_; });
        }
    }

    void scan()
    {
        snapshot_.refresh(mj_data_, sim_mutex_, true);
        mjData *d = snapshot_.data();
        mjtNum origin[3], quat[4];
        mount_.worldPose(d, origin, quat);

        // Ray directions (sensor frame) for this scan, continuing the R2 sequence
        // (Roberts' generalized golden ratio) so successive scans interleave.
        constexpr double g1 = 0.7548776662466927, g2 = 0.5698402909980532;
        for (int i = 0; i < npoints_; i++, seq_++) {
            double u = std::fmod(0.5 + g1 * static_cast<double>(seq_), 1.0);
            double w = std::fmod(0.5 + g2 * static_cast<double>(seq_), 1.0);
            double az = az0_ + az_span_ * u;
            double sel = sin_el0_ + sin_el_span_ * w;
            double cel = std::sqrt(std::fmax(0.0, 1 - sel * sel));
            mjtNum *dir = dir_.data() + 3 * i;
            dir[0] = cel * std::cos(az);
            dir[1] = cel * std::sin(az);
            dir[2] = sel;
            int ring = static_cast<int>((std::asin(sel) - el0_) / el_span_ * rings_);
            ring_[i] = static_cast<uint16_t>(std::min(std::max(ring, 0), rings_ - 1));
            mju_rotVecQuat(vec_.data() + 3 * i, dir, quat);
        }
        mj_multiRay(mj_model_, d, origin, vec_.data(), geomgroup_, 1, exclude_body_, geomid_.data(),
                    dist_.data(), npoints_, range_max_);

        auto &data = cloud_.data();
        data.resize(static_cast<size_t>(npoints_) * kPointStep);
        std::vector<uint8_t> *wdata = deskewed_pub_ ? &deskewed_.data() : nullptr;
        if (wdata) {
            wdata->resize(data.size());
        }
        const float dt = static_cast<float>(1.0 / hz_ / npoints_);
        uint32_t n = 0;
        for (int i = 0; i < npoints_; i++) {
            double r = dist_[i];
            if (r < 0 || r > range_max_) {
                continue;  // no return
            }
            if (noise_ > 0) {
                r += noise_dist_(rng_) * noise_;
            }
            if (r < range_min_ || r > range_max_) {
                continue;
            }
            const mjtNum *dir = dir_.data() + 3 * i;
            float xyz[3] = {static_cast<float>(dir[0] * r), static_cast<float>(dir[1] * r),
                            static_cast<float>(dir[2] * r)};
            float intensity = Intensity(geomid_[i]);
            float t = dt * i;
            WritePoint(data.data() + n * kPointStep, xyz, intensity, ring_[i], t);
            if (wdata) {
                const mjtNum *v = vec_.data() + 3 * i;
                float wxyz[3] = {static_cast<float>(origin[0] + v[0] * r), static_cast<float>(origin[1] + v[1] * r),
                                 static_cast<float>(origin[2] + v[2] * r)};
                WritePoint(wdata->data() + n * kPointStep, wxyz, intensity, ring_[i], t);
            }
            n++;
        }
        data.resize(static_cast<size_t>(n) * kPointStep);
        auto stamp = perception::WallStamp();
        Finish(cloud_, n, stamp);
        pub_->Write(cloud_);
        if (wdata) {
            wdata->resize(data.size());
            Finish(deskewed_, n, stamp);
            deskewed_pub_->Write(deskewed_);
        }

        auto &stats = perception::UtlidarStats::instance();
        float hz = rate_.tick();
        std::lock_guard<std::mutex> lock(stats.mtx);
        stats.cloud_frequency = hz;
        stats.cloud_size = n;
        stats.cloud_scan_num++;
    }

    static void Finish(Cloud_t &c, uint32_t n, const builtin_interfaces::msg::dds_::Time_ &stamp)
    {
        c.header().stamp(stamp);
        c.width(n);
        c.row_step(n * kPointStep);
    }

    static void WritePoint(uint8_t *p, const float xyz[3], float intensity, uint16_t ring, float t)
    {
        std::memset(p, 0, kPointStep);
        std::memcpy(p, xyz, 3 * sizeof(float));
        std::memcpy(p + 16, &intensity, sizeof(float));
        std::memcpy(p + 20, &ring, sizeof(uint16_t));
        std::memcpy(p + 24, &t, sizeof(float));
    }

    // Reflectivity stand-in: the luminance of the hit geom's colour, 0..255.
    float Intensity(int geom) const
    {
        if (geom < 0) {
            return 0;
        }
        int mat = mj_model_->geom_matid[geom];
        const float *rgba = mat >= 0 ? mj_model_->mat_rgba + 4 * mat : mj_model_->geom_rgba + 4 * geom;
        return 255.f * (0.299f * rgba[0] + 0.587f * rgba[1] + 0.114f * rgba[2]);
    }

    std::recursive_mutex *sim_mutex_;
    perception::KinematicsSnapshot snapshot_;
    perception::Mount mount_;
    double hz_ = 15;
    int npoints_ = 1440;
    int rings_ = 18;
    double noise_ = 0;
    double az0_ = 0, az_span_ = 0, sin_el0_ = 0, sin_el_span_ = 0, el0_ = 0, el_span_ = 0;
    double range_min_ = 0, range_max_ = 0;
    mjtByte geomgroup_[mjNGROUP] = {0};
    int exclude_body_ = -1;
    std::string frame_id_, deskewed_frame_id_;
    unsigned long long seq_ = 0;
    std::vector<mjtNum> dir_, vec_, dist_;
    std::vector<uint16_t> ring_;
    std::vector<int> geomid_;
    std::mt19937 rng_{12345};
    std::normal_distribution<double> noise_dist_{0.0, 1.0};
    perception::RateMeter rate_;
    Cloud_t cloud_, deskewed_;
    std::unique_ptr<Pub_t> pub_, deskewed_pub_;
    std::mutex stop_mtx_;
    std::condition_variable stop_cv_;
    bool stop_ = false;
    std::thread thread_;
};

// rt/utlidar/imu (sensor_msgs/Imu): the lidar's IMU. The base IMU sensors (imu_quat, imu_gyro,
// imu_acc) rotated into the lidar frame (lever-arm terms of the acceleration are ignored); the
// orientation is the lidar frame's orientation in the world. Params: site (default `utlidar`;
// must be on the same body as the IMU site), hz (250), frame_id (utlidar_imu), topic.
class UtlidarImuComponent : public LowRateComponent
{
    using Imu_t = unitree::robot::RealTimePublisher<sensor_msgs::msg::dds_::Imu_>;

public:
    UtlidarImuComponent(mjModel *model, mjData *data, const YAML::Node &params)
        : LowRateComponent(model, data, params, "utlidar_imu", 250.0), base_(model, "utlidar_imu")
    {
        using perception::Param;
        auto site_name = Param<std::string>(params, "site", "utlidar");
        int site = mj_name2id(model, mjOBJ_SITE, site_name.c_str());
        if (site < 0) {
            throw std::invalid_argument("utlidar_imu: no site named '" + site_name + "' in the model");
        }
        if (model->site_bodyid[site] != base_.body()) {
            throw std::invalid_argument("utlidar_imu: site '" + site_name + "' is not on the IMU's body");
        }
        // Rotation from the IMU site frame to the lidar site frame (both fixed in the same body).
        mjtNum imu_inv[4];
        mju_negQuat(imu_inv, model->site_quat + 4 * base_.site());
        mju_mulQuat(rel_, imu_inv, model->site_quat + 4 * site);
        mju_negQuat(rel_inv_, rel_);
        pub_ = std::make_unique<Imu_t>(Param<std::string>(params, "topic", "rt/utlidar/imu"));
        pub_->msg_.header().frame_id(Param<std::string>(params, "frame_id", "utlidar_imu"));
    }

    void step() override
    {
        if (!due() || !pub_->trylock()) {
            return;
        }
        mjtNum qs[4], gyro[3], acc[3], q[4], w[3], a[3];
        base_.imu(mj_data_, qs, gyro, acc);
        mju_mulQuat(q, qs, rel_);
        mju_rotVecQuat(w, gyro, rel_inv_);
        mju_rotVecQuat(a, acc, rel_inv_);
        auto &msg = pub_->msg_;
        msg.header().stamp(perception::WallStamp());
        perception::SetQuat(msg.orientation(), q);
        perception::SetVec(msg.angular_velocity(), w);
        perception::SetVec(msg.linear_acceleration(), a);
        pub_->unlockAndPublish();

        auto &stats = perception::UtlidarStats::instance();
        float hz = rate_.tick();
        std::lock_guard<std::mutex> lock(stats.mtx);
        stats.imu_frequency = hz;
        perception::QuatToRpy(q, stats.imu_rpy);
    }

private:
    perception::BaseState base_;
    mjtNum rel_[4], rel_inv_[4];
    perception::RateMeter rate_;
    std::unique_ptr<Imu_t> pub_;
};

// rt/utlidar/robot_odom (nav_msgs/Odometry) and rt/utlidar/robot_pose (geometry_msgs/PoseStamped):
// the base_link pose in the world (ground truth, from the IMU-site sensors also used for
// rt/sportmodestate). Odometry twist is in the child frame. Params: hz (50), frame_id (odom),
// child_frame_id (base_link, odometry only), topic.
class UtlidarOdomComponent : public LowRateComponent
{
    using Odom_t = unitree::robot::RealTimePublisher<nav_msgs::msg::dds_::Odometry_>;
    using Pose_t = unitree::robot::RealTimePublisher<geometry_msgs::msg::dds_::PoseStamped_>;

public:
    enum Kind { kOdom, kPose };

    UtlidarOdomComponent(mjModel *model, mjData *data, const YAML::Node &params, Kind kind)
        : LowRateComponent(model, data, params, kind == kOdom ? "utlidar_robot_odom" : "utlidar_robot_pose", 50.0),
          base_(model, kind == kOdom ? "utlidar_robot_odom" : "utlidar_robot_pose")
    {
        using perception::Param;
        auto frame = Param<std::string>(params, "frame_id", "odom");
        if (kind == kOdom) {
            odom_ = std::make_unique<Odom_t>(Param<std::string>(params, "topic", "rt/utlidar/robot_odom"));
            odom_->msg_.header().frame_id(frame);
            odom_->msg_.child_frame_id(Param<std::string>(params, "child_frame_id", "base_link"));
        } else {
            pose_ = std::make_unique<Pose_t>(Param<std::string>(params, "topic", "rt/utlidar/robot_pose"));
            pose_->msg_.header().frame_id(frame);
        }
    }

    void step() override
    {
        if (!due()) {
            return;
        }
        mjtNum p[3], q[4], v[3], w[3];
        base_.bodyState(mj_data_, p, q, v, w);
        auto setPose = [&](geometry_msgs::msg::dds_::Pose_ &pose) {
            pose.position().x(p[0]);
            pose.position().y(p[1]);
            pose.position().z(p[2]);
            perception::SetQuat(pose.orientation(), q);
        };
        if (odom_ && odom_->trylock()) {
            auto &msg = odom_->msg_;
            msg.header().stamp(perception::WallStamp());
            setPose(msg.pose().pose());
            mjtNum qinv[4], vb[3], wb[3];
            mju_negQuat(qinv, q);
            mju_rotVecQuat(vb, v, qinv);
            mju_rotVecQuat(wb, w, qinv);
            perception::SetVec(msg.twist().twist().linear(), vb);
            perception::SetVec(msg.twist().twist().angular(), wb);
            odom_->unlockAndPublish();
        }
        if (pose_ && pose_->trylock()) {
            pose_->msg_.header().stamp(perception::WallStamp());
            setPose(pose_->msg_.pose());
            pose_->unlockAndPublish();
        }
    }

private:
    perception::BaseState base_;
    std::unique_ptr<Odom_t> odom_;
    std::unique_ptr<Pose_t> pose_;
};

// rt/utlidar/foot_position (sensor_msgs/PointCloud2): one point (x, y, z float32, point_step 12)
// per foot, in the world frame `frame_id`. Foot bodies are posed on a private mjData refreshed on
// the bridge thread with try_lock (a busy physics mutex just delays the update by a tick).
// Params: bodies (FR_foot, FL_foot, RR_foot, RL_foot: the Unitree leg order), hz (50),
// frame_id (odom), topic.
class UtlidarFootPositionComponent : public LowRateComponent
{
    using Cloud_t = unitree::robot::RealTimePublisher<sensor_msgs::msg::dds_::PointCloud2_>;

public:
    UtlidarFootPositionComponent(mjModel *model, mjData *data, const YAML::Node &params, const BridgeContext &ctx)
        : LowRateComponent(model, data, params, "utlidar_foot_position", 50.0), sim_mutex_(ctx.sim_mutex),
          snapshot_(model)
    {
        using perception::Field;
        using perception::Param;
        auto names = Param(params, "bodies", std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"});
        for (const auto &n : names) {
            int id = mj_name2id(model, mjOBJ_BODY, n.c_str());
            if (id < 0) {
                throw std::invalid_argument("utlidar_foot_position: no body named '" + n + "' in the model");
            }
            bodies_.push_back(id);
        }
        pub_ = std::make_unique<Cloud_t>(Param<std::string>(params, "topic", "rt/utlidar/foot_position"));
        auto &c = pub_->msg_;
        c.header().frame_id(Param<std::string>(params, "frame_id", "odom"));
        c.height(1);
        c.width(static_cast<uint32_t>(bodies_.size()));
        c.fields({Field("x", 0, perception::kFloat32), Field("y", 4, perception::kFloat32),
                  Field("z", 8, perception::kFloat32)});
        c.point_step(12);
        c.row_step(12 * c.width());
        c.is_dense(true);
        c.data().resize(c.row_step());
    }

    void step() override
    {
        pending_ = due() || pending_;
        if (!pending_ || !snapshot_.refresh(mj_data_, sim_mutex_, false) || !pub_->trylock()) {
            return;
        }
        pending_ = false;
        const mjData *d = snapshot_.data();
        auto &c = pub_->msg_;
        c.header().stamp(perception::WallStamp());
        for (size_t i = 0; i < bodies_.size(); i++) {
            float xyz[3];
            for (int k = 0; k < 3; k++) {
                xyz[k] = static_cast<float>(d->xpos[3 * bodies_[i] + k]);
            }
            std::memcpy(c.data().data() + 12 * i, xyz, sizeof(xyz));
        }
        pub_->unlockAndPublish();
    }

private:
    std::recursive_mutex *sim_mutex_;
    perception::KinematicsSnapshot snapshot_;
    std::vector<int> bodies_;
    bool pending_ = false;
    std::unique_ptr<Cloud_t> pub_;
};

// rt/utlidar/lidar_state (unitree_go::LidarState_): status of the simulated lidar. cloud_* and
// imu_* come from the utlidar_cloud / utlidar_imu components (0 when they are not loaded),
// versions read "sim", error_state 0. Params: hz (1), topic.
class UtlidarLidarStateComponent : public LowRateComponent
{
    using State_t = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::LidarState_>;

public:
    UtlidarLidarStateComponent(mjModel *model, mjData *data, const YAML::Node &params)
        : LowRateComponent(model, data, params, "utlidar_lidar_state", 1.0)
    {
        pub_ = std::make_unique<State_t>(perception::Param<std::string>(params, "topic", "rt/utlidar/lidar_state"));
        pub_->msg_.firmware_version("sim");
        pub_->msg_.software_version("sim");
        pub_->msg_.sdk_version("sim");
    }

    void step() override
    {
        if (!due() || !pub_->trylock()) {
            return;
        }
        auto &msg = pub_->msg_;
        msg.stamp(perception::WallSeconds());
        {
            auto &stats = perception::UtlidarStats::instance();
            std::lock_guard<std::mutex> lock(stats.mtx);
            msg.cloud_frequency(stats.cloud_frequency);
            msg.cloud_size(stats.cloud_size);
            msg.cloud_scan_num(stats.cloud_scan_num);
            msg.imu_frequency(stats.imu_frequency);
            for (int i = 0; i < 3; i++) {
                msg.imu_rpy()[i] = stats.imu_rpy[i];
            }
        }
        pub_->unlockAndPublish();
    }

private:
    std::unique_ptr<State_t> pub_;
};

inline void RegisterPerceptionComponents()
{
    auto &registry = ComponentRegistry::instance();
    registry.add("utlidar_cloud", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &c) { return std::make_unique<UtlidarCloudComponent>(m, d, p, c); });
    registry.add("utlidar_imu", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<UtlidarImuComponent>(m, d, p); });
    registry.add("utlidar_robot_odom", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<UtlidarOdomComponent>(m, d, p, UtlidarOdomComponent::kOdom); });
    registry.add("utlidar_robot_pose", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<UtlidarOdomComponent>(m, d, p, UtlidarOdomComponent::kPose); });
    registry.add("utlidar_foot_position", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &c) { return std::make_unique<UtlidarFootPositionComponent>(m, d, p, c); });
    registry.add("utlidar_lidar_state", [](mjModel *m, mjData *d, const YAML::Node &p, const BridgeContext &) { return std::make_unique<UtlidarLidarStateComponent>(m, d, p); });
}
