#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "CameraPoseVisualization.h"
#include <eigen3/Eigen/Dense>
#include "../estimator.h"
#include "../parameters.h"
#include <fstream>

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern ros::Publisher pub_pose_graph;
extern int IMAGE_ROW, IMAGE_COL;
/**
 * @brief register ROS publisher
 * 
 * @param n 
 */
void registerPub(ros::NodeHandle &n);
/**
 * @brief publish imu one step propogation in nav_msgs::odometry
 * 
 * @param P: position 
 * @param Q: quaternion
 * @param V: velocity 
 * @param header 
 */
void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param t 
 */
void printStatistics(const Estimator &estimator, double t);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 * @param header 
 */
void pubTF(const Estimator &estimator, const std_msgs::Header &header);
/**
 * @brief 
 * 
 * @param estimator 
 */
void pubKeyframe(const Estimator &estimator);
/**
 * @brief 
 * 
 * @param estimator 
 */
void pubRelocalization(const Estimator &estimator);