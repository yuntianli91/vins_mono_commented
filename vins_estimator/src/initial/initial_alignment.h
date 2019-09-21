/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-06 16:51:55
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-21 10:07:00
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

/**
 * @brief 
 * 
 */
class ImageFrame
{
    public:
        /**
         * @brief Default constructor
         * 
         */
        ImageFrame(){};
        /**
         * @brief Construct a new Image Frame object
         * 
         * @param _points 
         * @param _t 
         */
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };

        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points; ///
        double t; ///
        Matrix3d R; /// R_{b_k}^{c_0}
        Vector3d T; /// p_{c_k}^{c_0}
        IntegrationBase *pre_integration; /// IMU preintegrations for frames [k,k+1]
        bool is_key_frame; /// falg for whether current frame is keyframe
};
/**
 * @brief IMU and camera alignment 
 * 
 * @param all_image_frame : map container of all frames in sliding window (map<double, ImageFrame>)
 * @param Bgs : initialized gyro bias 
 * @param g : initialized gravity vector
 * @param x : initialized state vector
 * @return true : initialization succeed
 * @return false : initialization failed
 */
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);