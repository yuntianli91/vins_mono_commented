/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-08-28 09:07:33
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-08-28 20:20:46
 */
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;// pose estimator object

std::condition_variable con; // conditional variable for sleep and wake thread
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf; // buffer if imu_msg ptr
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;
// mutex lock
std::mutex m_buf; // lock for all data buffer operations
std::mutex m_state; // lock for imu state prediction operations
std::mutex i_buf; // lock for 
std::mutex m_estimator; // lock for estimator operations

double latest_time; // time of last imu measurement
Eigen::Vector3d tmp_P; // temporary position
Eigen::Quaterniond tmp_Q; // temporary quaternion
Eigen::Vector3d tmp_V; // temporary linear velocity 
Eigen::Vector3d tmp_Ba; // temporary accelerometer bias
Eigen::Vector3d tmp_Bg; // temporary gyroscope bias
Eigen::Vector3d acc_0; // acceleration of last imu measurement
Eigen::Vector3d gyr_0; // gyroscope of last imu measurement
bool init_feature = 0; // num of initiate features
bool init_imu = 1;
double last_imu_t = 0; // timestamp of last imu measurement

/**
 * @brief pose prediction from imu measurement using median integral
 * 
 * @param imu_msg: sensor_msgs::ImuConstPtr of imu_msg
 */
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();// timestamp of current imu message
    // if it is not the first IMU message, initiate it.
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    // obtain imu measurement period and reset latest_time
    double dt = t - latest_time;
    latest_time = t;
    
    // obtain current linear acceleration
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    // obtain current angular velocity
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // acceleration of last imu measurement in world frame
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;
    // average angular velocty used for median integral
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    // update quaternion
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    // acceleration of current imu measurement in world frame
    // *notice: quaternion has been updated!!!
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    // average linear acceleration used for median integral
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    // update position and velocity
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc; // temporary position
    tmp_V = tmp_V + dt * un_acc; // temporary linear velocity

    // reset acc_0 and gyr_0 using current measurement
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
/**
 * @brief 
 * 
 */
void update()
{
    TicToc t_predict;// timer object for calculating duration
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    // measurement = one img plus several imu
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        // if there is no data neither in imu_buf or feature_buf, return empty measurement.
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;
        // if timestamp of latest imu measurement is smaller than the timestamp  of first img which means
        // there is no imu measurement between two frames  
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
        // if timestamp of first imu measurement is larger than the timestamp of first img which means
        // there will be some small interval without any imu measurement after img
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            // pop the first img 
            feature_buf.pop();
            continue;
        }

        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        // add several imu measurements before img
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());// emplace_back is equal to push_back except some special cases
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());// add the first imu measurement after img
        // make sure IMUs is not empty
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}
/**
 * @brief callback function each time an imu_msgs arrived, including:
 *  a) saving ImuConstPtr to imu_buf;
 *  b) calculating one step prediction of imu and publish to "imu_propogation" 
 * 
 * @param imu_msg: sensor_msgs::ImuConstPtr of imu_msg
 */
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // check whether imu message arrived in right order
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    // set last_imu_t to current_imu_t
    last_imu_t = imu_msg->header.stamp.toSec();
    // lock imu_buf when main thread are reading and saving 
    // imu message ptr to imu_buf
    m_buf.lock();
    imu_buf.push(imu_msg);
    // unlock imu_buf and wake up process thread
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();// why again ?

    {
        // lock data with lock_guard, lock will be auto unlock after bracks
        std::lock_guard<std::mutex> lg(m_state);
        // calculate imu propogation
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        // pub imu propogation
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);// publish message
    }
}
/**
 * @brief operations each time received frature_msg
 * 
 * @param feature_msg 
 */
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}
/**
 * @brief operations when receive restart msg, including empty imu_buf and feature_buf,
 *  clear estimator state and reset all variables to initial value.
 * 
 * @param restart_msg: std_msgs::BoolConstPtr 
 */
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        // empty buffer
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        // reset estimator
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}
/**
 * @brief callback function each time a pointcloud msg arrived, including saving PointCloudConstPtr to relo_buf.
 * 
 * @param points_msg: sensor_msgs::PointCloudConstPtr 
 */
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

/**
 * @brief thread function of visual inertial odometry
 * 
 */
void process()
{
    while (true)
    {
        // vector of pairs contain (vector<ImuConstPtr>, PointCloudConstPtr)
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);/// lock all data buffer when reading measurements
        // keep block this
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock(); // unlock data buffer for receiving msg when gerMeasurements finished
        
        m_estimator.lock();// lock variables related to estimator
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;// ? img represent by pointcloud ?
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec(); // imu time
                double img_t = img_msg->header.stamp.toSec() + estimator.td; // img time
                // deal with imu measurements before img time
                if (t <= img_t)
                { 
                    // initiation
                    if (current_time < 0)
                        current_time = t;
                    
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);// make sure the right order, dt should always be positive
                    current_time = t;// reset current time
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    // imu preintegration
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                // deal with imu measurement after img time (should be only one measurement)
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");/// initiate estimator node
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);/// change ros log output level to 'Info'.
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");
    // register publisher topics
    registerPub(n);
    // message subsription function
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());// enablr TCP_NODELAY ti minimum delay
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
