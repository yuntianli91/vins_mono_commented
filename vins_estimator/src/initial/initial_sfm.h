/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-02 09:27:35
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-06 16:50:43
 */
#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;


/**
 * @brief structure that represent one SFM feature
 * 
 */
struct SFMFeature
{
    bool state; /// triangulated/tracked state
    int id; /// id of feature
    vector<pair<int,Vector2d>> observation; ///vector of all observations in all frames
    double position[3]; /// position in world frame
    double depth; /// depth of point in world frame
};

/**
 * @brief structure of repeojection error
 * 
 */
struct ReprojectionError3D
{
	/**
	 * @brief Construct a new Reprojection Error 3 D object
	 * 
	 * @param observed_u : observed pixel coordinate 
	 * @param observed_v : observed pixel coordinate
	 */
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u_(observed_u), observed_v_(observed_v)
		{}

	/**
	 * @brief functor for residual calculation 
	 * 
	 * @tparam T 
	 * @param camera_R : array of quaternion q_c^w(R_w^c)
	 * @param camera_T : array of translation t_w^c
	 * @param point : array of point coordinates in world frame
	 * @param residuals : reprojection error
	 * @return true : succeed
	 * @return false :failed
	 */
	template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
	{
		T p[3];
		// QuaternionRotatePoint(q, pt, result):rotate pt with R(q)*pt
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
		T xp = p[0] / p[2];
    	T yp = p[1] / p[2];
    	residuals[0] = xp - T(observed_u_);
    	residuals[1] = yp - T(observed_v_);
    	return true;
	}
	/**
	 * @brief factory function to create CostFunction object
	 * 
	 * @param observed_u_： observed pixel coordinate 
	 * @param observed_v_: observed pixel coordinate
	 * @return ceres::CostFunction* 
	 */
	static ceres::CostFunction* Create(const double observed_u,
	                                   const double observed_v) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_u,observed_v)));
	}

	double observed_u_; /// observed pixel coordinate
	double observed_v_; /// observed pixel coordinate
};

class GlobalSFM
{
public:
	/**
	 * @brief default constructor, construct a new empty GlobalSFM object
	 * 
	 */
	GlobalSFM();
	/**
	 * @brief processing GlobalSFM
	 * 
	 * @param frame_num: num of all frames in sliding window 
	 * @param q: point of array contains all rotation quaternion from world to ith frame
	 * @param T: point of array contains all translation from ith frame to world 
	 * @param l: index of first frame in sliding window 
	 * @param relative_R: initial guess of R from last frame to frst frame in sliding window
	 * @param relative_T: initial guess of t from last frame to first frame in sliding window 
	 * @param sfm_f: vector contains all feature points 
	 * @param sfm_tracked_points: map contains tracked feature points and their id 
	 * @return true 
	 * @return false 
	 */
	bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

private:
	/**
	 * @brief solve pose with PnP
	 * 
	 * @param R_initial: initial guess R of frame i (R_w^{C_i})
	 * @param P_initial: initial guess t of frame i (t_w^{C_i})
	 * @param i: current frame index 
	 * @param sfm_f: vector contains SFMFrature 
	 * @return true: solve pnp succeed
	 * @return false: solve pnp failed 
	 */
	bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);
	/**
	 * @brief calculating 3D coordinates of point in world frame. 
	 * 	3D coordinates of point: X=[X,Y,Z,1]
	 * 	corresponding normalized 2D coordinates in camera frame: x=[x,y,1]
	 * 	pose of camera frame: T=[t1;t2;t3]
	 *  depth of point in camera frame: s
	 * 	then we have sx=TX => [sx]^TX=0. Here, "^" is the symbol for symmetric matrix.
	 * 	this will be an over constrain linear equations which can bo solved with SVD decomposition
	 *  when we have more then one point pairs.
	 * 
	 * @param Pose0: pose of first frame(T_1) 
	 * @param Pose1: pose of second frame(T_2) 
	 * @param point0: normalized 2D coordinates of 3D point in camera frame 1 
	 * @param point1: normalized 2D coordinates of 3D point in camera frame 2
	 * @param point_3d: 3D coordinates of point 
	 */
	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
							Vector2d &point0, Vector2d &point1, Vector3d &point_3d);

	/**
	 * @brief triangulated feature points with observations from two frames
	 * 
	 * @param frame0: index of first frame 
	 * @param Pose0: pose of first frame 
	 * @param frame1: index of second frame 
	 * @param Pose1: pose of second frame 
	 * @param sfm_f: vector contains all feature points 
	 */
	void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
							  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
							  vector<SFMFeature> &sfm_f);

	int feature_num;// total number of features in current scene 
};