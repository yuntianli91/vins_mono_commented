/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-17 10:35:46
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-25 16:27:00
 */
#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

/**
 * @brief check whether point are inside image border, image border are 1 pixel
 * 
 * @param pt : pixel coordinates of point 
 * @return true : inside border
 * @return false : out of border 
 */
bool inBorder(const cv::Point2f &pt); 

/**
 * @brief : reduce vector of all 2D points to vector of tracked 2D points 
 * 
 * @param v : vector of all 2D points 
 * @param status : vector of all 2D points' status (tracked or lost)
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status); 

/**
 * @brief : reduce vector of all 2D points indices to vector of tracked 2D points indices 
 * 
 * @param v : vector of all 2D points indices 
 * @param status : vector of all 2D points' status (tracked or lost)
 */
void reduceVector(vector<int> &v, vector<uchar> status);

/**
 * @brief calss of feature tracker
 * 
 */
class FeatureTracker
{
  public:
    /**
     * @brief Construct a new Feature Tracker object
     * 
     */
    FeatureTracker(); ///构造函数

    /**
     * @brief : read image from dataset
     * 
     * @param _img : current image from IMAGE_TOPIC
     * @param _cur_time : time_stamp inside IMAGE_MSG
     */
    void readImage(const cv::Mat &_img,double _cur_time);

    /**
     * @brief Set the Mask which all tracked pointed are cover with filled black circle
     *  
     * 
     */
    void setMask();

    /**
     * @brief add new extracted points to frow_pts and ids and track_cnt. 
     *  ids will be set to -1, then updateID function will update to it actually id in totally extracted points 
     *  track_cnt will be set to 1;
     */
    void addPoints();

    /**
     * @brief update id of ith tracked point
     * 
     * @param i 
     * @return true 
     * @return false 
     */
    bool updateID(unsigned int i);

    /**
     * @brief read instrinsic matrix from YAML file 
     * 
     * @param calib_file 
     */
    void readIntrinsicParameter(const string &calib_file);

    /**
     * @brief show undistortion image
     * 
     * @param name 
     */
    void showUndistortion(const string &name);

    /**
     * @brief reject tracked point pairs which are outliers for current fundamental matrix
     * 
     */
    void rejectWithF();

    /**
     * @brief 
     * 
     */
    void undistortedPoints();

    cv::Mat mask; /// mask image with all tracked points covered will filled black circle
    cv::Mat fisheye_mask; /// mask for fisheye camera
    cv::Mat prev_img, cur_img, forw_img; /// img 
    vector<cv::Point2f> n_pts; /// new extracted points from forw img.
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts; /// points in prev_ing, cur_img and forw_img
    vector<cv::Point2f> prev_un_pts, cur_un_pts; ///undistorted points from prev_img and cur_img 
    vector<cv::Point2f> pts_velocity;
    vector<int> ids; /// vector contains ids of corresponding feature points 
    vector<int> track_cnt; /// vector contains tracked times of corresponding feature points
    map<int, cv::Point2f> cur_un_pts_map; /// map container of undistorted points in cur_imgm key is the id of point
    map<int, cv::Point2f> prev_un_pts_map; /// map container of undistorted points in prev_img, key is the id of point
    camodocal::CameraPtr m_camera;
    double cur_time; /// time_stamp of frow_img
    double prev_time; /// time_stamp of cur_img

    static int n_id; /// static member, num of totally extracted points
};
