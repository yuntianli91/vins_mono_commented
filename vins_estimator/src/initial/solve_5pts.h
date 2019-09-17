/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-01 14:00:30
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-01 14:00:30
 */
#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>

class MotionEstimator
{
  public:
    /**
     * @brief calculate relative R and t
     * 
     * @param corres : correspoinding feature points pair<c_k,c_{k+1}>
     * @param R : relative R from c_{k+1} to c_k;
     * @param T : relative t from c_{k+1} to c_k;
     * @return true 
     * @return false 
     */
    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

  private:
    /**
     * @brief test how many 
     * 
     * @param l 
     * @param r 
     * @param R 
     * @param t 
     * @return double 
     */
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    /**
     * @brief decompose E to find R and t candidates
     * 
     * @param E 
     * @param R1 
     * @param R2 
     * @param t1 
     * @param t2 
     */
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};


