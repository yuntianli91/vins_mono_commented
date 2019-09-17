/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-06 16:51:55
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-08 10:11:34
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

        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points; ///
        double t; ///
        Matrix3d R; ///
        Vector3d T; ///
        IntegrationBase *pre_integration; ///
        bool is_key_frame; ///
};
/**
 * @brief IMU and camera alignment 
 * 
 * @param all_image_frame : map of all frames in sliding window (map<double, ImageFrame>)
 * @param Bgs 
 * @param g 
 * @param x 
 * @return true 
 * @return false 
 */
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);