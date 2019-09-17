/*
 * @Description: caliberate rotation matrix between camera and IMU
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-08-30 14:49:22
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-07 15:10:55
 */
#include "initial_ex_rotation.h"

/**
 * @brief Construct a new InitialEXRotation object and set all rotation matrix to Identity matrix
 * 
 */
InitialEXRotation::InitialEXRotation(){
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}
/**
 * @brief caliberate extrinsic matrix
 * 
 * @param corres: all corresponding point in two frames 
 * @param delta_q_imu: rotation quaternion from b_{k+1} to b_k 
 * @param calib_ric_result 
 * @return true 
 * @return false 
 */
bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    // eigen use rotation quaternion
    frame_count++;
    Rc.push_back(solveRelativeR(corres));//c_k->c_k+1
    Rimu.push_back(delta_q_imu.toRotationMatrix());//R_{b_k}^{b_{k+1}}
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

    Eigen::MatrixXd A(frame_count * 4, 4);//Q_n
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG(
            "%d %f", i, angular_distance);
        // huber coefficient "w_i"
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;

        ++sum_ok;
        Matrix4d L, R;

        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);// calculate SVD of Q_n with square U and square V
    Matrix<double, 4, 1> x = svd.matrixV().col(3);//last com of V which is the vector correspond to the minimum singular value
    Quaterniond estimated_R(x);//estimated rotation quaternion q_c^b
    ric = estimated_R.toRotationMatrix().inverse();
    //cout << svd.singularValues().transpose() << endl;
    //cout << ric << endl;
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();//get last three elements of singularValue vector
    // termination Criteria
    if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)//(why 0.25 ?)
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}
/**
 * @brief solve rotation matrix from 3D points
 * 
 * @param corres: vector contain pairs of correspoing points in homogeneous coordinates, minimum size of this vector is 9
 * @return Matrix3d: rotation matrix 
 */
Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat E = cv::findFundamentalMat(ll, rr);// fundamental or essential ?
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        // choose the rotation matrix with maximum
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;
        // convert from cv::Mat to Eigen::Matrix3d
        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}
/**
 * @brief test triangulation to select the right R and t
 * 
 * @param l: points in left frame(first frame) 
 * @param r: points in right frame(second frame)
 * @param R: choosed rotation matrix R 
 * @param t: choosed translation t 
 * @return double percentage of points with both positive depth
 */
double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    // 4xN array of reconstructed points in homogeneous coordinates
    cv::Mat pointcloud;
    // transformation matrix of first frame
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    // transformation matrix of second frame
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    // triangulate points
    cv::triangulatePoints(P, P1, l, r, pointcloud);
    // counter for counting points with both positive depth
    int front_count = 0;
    //
    for (int i = 0; i < pointcloud.cols; i++)
    {
        // for normalizing coordinate with z = 1
        double normal_factor = pointcloud.col(i).at<float>(3);
        // reproject to first camera frame
        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
            front_count++;
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}
/**
 * @brief recover R and t from Essential Matrix
 * 
 * @param E: Essential Matrix 
 * @param R1: first R
 * @param R2: first R
 * @param t1: first t 
 * @param t2: second t 
 */
void InitialEXRotation::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    // E = U*Sigma*V^T
    cv::SVD svd(E, cv::SVD::MODIFY_A);

    // Rz(-pi/2)
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    // Rz(pi/2)
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    // R = U*W*V^T
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    // when Sigma = diag[1, 1, 0], t = U(3);
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}
