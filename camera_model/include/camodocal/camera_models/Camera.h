/**
 * @file Camera.h
 * @author yuntianli (yuntianlee91@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camodocal
{
/**
 * @brief base class for all camera model
 * 
 */
class Camera
{
public:
    // align Eigen ptr  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// enumerate variable of camera model
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE,
        SCARAMUZZA
    };
    /**
     * @brief nested class for camera parameters
     * 
     */
    class Parameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Construct a new Parameters object
         * 
         * @param modelType: model of current camera object
         */
        Parameters(ModelType modelType);

        /**
         * @brief Construct a new Parameters object
         * 
         * @param modelType: model of current camera object
         * @param cameraName: name of current camera object
         * @param w: image width of current camera object
         * @param h: image height of current camera object
         */
        Parameters(ModelType modelType, const std::string& cameraName,
                   int w, int h);

        /**
         * @brief return modelType of current camera object
         * 
         * @return: 
         *  @retval modelType: modelType of current camera object
         */
        ModelType& modelType(void);

        /**
         * @brief return name of current camera object
         * 
         * @return:
         *  @retval cameraName: std::string type 
         */
        std::string& cameraName(void);

        /**
         * @brief return image width of current camera object
         * 
         * @return:
         *  @retval imageWidth: image width in pixels, int type
         */
        int& imageWidth(void);

        /**
         * @brief return image height of current camera object 
         * 
         * @return:
         *  @retval imageHeight: image height in pixels, int type
         */
        int& imageHeight(void);

        /**
         * @brief return modelType of current camera object (const data member)
         * 
         * @return: 
         *  @retval modelType: modelType of current camera object
         */
        ModelType modelType(void) const;
 
         /**
         * @brief return name of current camera object(const data member)
         * 
         * @return:
         *  @retval cameraName: std::string type 
         */
        const std::string& cameraName(void) const;
  
         /**
         * @brief return image width of current camera object(const data member)
         * 
         * @return:
         *  @retval imageWidth: image width in pixels, int type
         */
        int imageWidth(void) const;
  
         /**
         * @brief return image height of current camera object (const data member)
         * 
         * @return:
         *  @retval imageHeight: image height in pixels, int type
         */
        int imageHeight(void) const;

        /**
         * @brief return number of camera intrinsics(const data member)
         * 
         * @return:
         *  @retval nIntrinsics: number of intrinsics, int type
         */
        int nIntrinsics(void) const;

        /**
         * @brief read parameters from YAML or XML file, (pure virtual function which must be override in son classes, following virtual function
         * with '=0' are all pure virtual function)
         * 
         * @param filename: filename of YAML or XML file
         */
        virtual bool readFromYamlFile(const std::string& filename) = 0;

        /**
         * @brief write parameters to YAML or XML file; 
         * 
         * @param filename: filename of YAML of XML file
         */
        virtual void writeToYamlFile(const std::string& filename) const = 0;

    protected:
        // camera modelType list in enum
        ModelType m_modelType;
        /// number of camera Intrinsics, for pinhole camera model are
        /// four distortion parameters: k1, k2, p1, p2
        /// four projection parameters: fx, fy, cx, cy
        int m_nIntrinsics;
        /// camera name
        std::string m_cameraName;
        /// image width in pixels
        int m_imageWidth;
        /// image height in pixels
        int m_imageHeight;
    };

    /// virtual type of function modelType
    virtual ModelType modelType(void) const = 0;
    /// virtual type of funtion cameraName
    virtual const std::string& cameraName(void) const = 0;
    /// virtual type of function imageWidth
    virtual int imageWidth(void) const = 0;
    /// virtual type of function imageHeight
    virtual int imageHeight(void) const = 0;
    /// virtual function of image Mask
    virtual cv::Mat& mask(void);
    /// virtual function of image Mask
    virtual const cv::Mat& mask(void) const;
     
    /// virtual function of camera intrinsics
    virtual void estimateIntrinsics(const cv::Size& boardSize,
                                    const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                    const std::vector< std::vector<cv::Point2f> >& imagePoints) = 0;
    /**
     * @brief calculate extrinsics with unit intrinsics
     * 
     * @param objectPoints: 3D points on object
     * @param imagePoints: 2D point on image plane
     * @param rvec: rotation vector
     * @param tvec: transformation vector
     */
    virtual void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                                    const std::vector<cv::Point2f>& imagePoints,
                                    cv::Mat& rvec, cv::Mat& tvec) const;

    /// Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    /// Lift points from the image plane to the projective space
    virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    /// Projects 3D points to the image plane (Pi function)
    virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const = 0;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    //virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                          Eigen::Matrix<double,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;
    //%output p

    //virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const = 0;
    virtual cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                            float fx = -1.0f, float fy = -1.0f,
                                            cv::Size imageSize = cv::Size(0, 0),
                                            float cx = -1.0f, float cy = -1.0f,
                                            cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;
    /// pure virtual function of parameter count
    virtual int parameterCount(void) const = 0;

    /// pure virtual function of reading parameters
    virtual void readParameters(const std::vector<double>& parameters) = 0;
    /// pure virtual function of writing parameters
    virtual void writeParameters(std::vector<double>& parameters) const = 0;
    /// pure virtual function of writing parameters to YAML file
    virtual void writeParametersToYamlFile(const std::string& filename) const = 0;
    /// pure virtual of converting parameters to string
    virtual std::string parametersToString(void) const = 0;

    /**
     * @brief Calculates the reprojection distance between points
     *
     * @param P1 first 3D point coordinates
     * @param P2 second 3D point coordinates
     * @return euclidean distance in the plane
     */
    double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

    /**
     * @brief calculate average reprojection error of all points in all frames (total error divede by total points) with 3D points and 2D points
     * 
     * @param objectPoints: 3D coordinates of object points
     * @param imagePoints: 2D coordinates of image points
     * @param rvecs: rotation vectors
     * @param tvecs: transformation vectors
     * @param perViewErrors:
     * @return:
     */
    double reprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                             const std::vector< std::vector<cv::Point2f> >& imagePoints,
                             const std::vector<cv::Mat>& rvecs,
                             const std::vector<cv::Mat>& tvecs,
                             cv::OutputArray perViewErrors = cv::noArray()) const;
    /**
     * @brief calculate reprojection error of one 3D point with camera pose P & Q
     * 
     * @param P: 3D coordinates of point
     * @param camera_q: quaternion of camera pose
     * @param camera_t: translation of camera pose
     * @param observed_p: corresponding image point
     * @return:
     */
    double reprojectionError(const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& camera_q,
                             const Eigen::Vector3d& camera_t,
                             const Eigen::Vector2d& observed_p) const;
    /**
     * @brief project 3D points to 2d plane
     * 
     * @param objectPoints:
     * @param rvec:
     * @param tvec:
     * @param imagePoints:
     */
    void projectPoints(const std::vector<cv::Point3f>& objectPoints,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       std::vector<cv::Point2f>& imagePoints) const;
protected:
    /// image mask
    cv::Mat m_mask;
};
/// cameraPtr
typedef boost::shared_ptr<Camera> CameraPtr; ///宏定义一个Camera类型的智能指针CameraPtr，shared_ptr不用手动去释放资源，它会智能地在合适的时候去自动释放。
/// CameraConstPtr
typedef boost::shared_ptr<const Camera> CameraConstPtr;

}

#endif
