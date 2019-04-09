/*
 * File:   Feature3DEngine.h
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */

#ifndef RGBD_ODOMETRY_H
#define RGBD_ODOMETRY_H

// Standard C++ includes
#include <string>
#include <iostream>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>

#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <rgbd_odometry/rgbd_odometry_core.h>

#define NUMIDCHARS 3

namespace stdpatch {

template < typename T > std::string to_string(const T& n) {
  std::ostringstream stm;
  stm << std::setw(NUMIDCHARS) << std::setfill('0') << n;
  //stm << n;
  return stm.str();
}
}

class RGBDOdometryEngine : public RGBDOdometryCore {
public:
    typedef boost::shared_ptr<RGBDOdometryEngine> Ptr;

    RGBDOdometryEngine() :
    RGBDOdometryCore(),
    nodeptr(new ros::NodeHandle),
    nh("~") {
        std::string opencl_path, depthmask_cl, tf_truth_topic, calibration_pose;
        std::string optical_parent, optical_frame, depth_processing_str;
        std::string feature_detector, feature_descriptor;
        bool useOpenCL, tf_truth_initialize;
        previous_time = ros::Time::now();
        nh.param<std::string>("OpenCL_path", opencl_path, ".");
        nh.param<std::string>("depthmask_cl", depthmask_cl, "depthmask.cl");
        nh.param("useOpenCL", useOpenCL, false);
        getImageFunctionProvider()->initialize(useOpenCL, opencl_path, depthmask_cl);

        nh.param<std::string>("feature_detector", feature_detector, "ORB");
        nh.param<std::string>("feature_descriptor", feature_descriptor, "ORB");

        rmatcher->setFeatureDetector(feature_detector);
        rmatcher->setDescriptorExtractor(feature_descriptor);

        nh.param<bool>("verbose", VERBOSE, "false");
        nh.param<bool>("visualize", DUMP_MATCH_IMAGES, "false");

        nh.param<std::string>("depth_processing", depth_processing_str, "none");

        if (depth_processing_str.compare("moving_average") == 0) {
            std::cout << "Applying moving average depth filter." << std::endl;
            this->depth_processing = Depth_Processing::MOVING_AVERAGE;
        } else if (depth_processing_str.compare("dither") == 0) {
            std::cout << "Applying dithering depth filter." << std::endl;
            this->depth_processing = Depth_Processing::DITHER;
        } else {
            this->depth_processing = Depth_Processing::NONE;
        }
    }

    virtual ~RGBDOdometryEngine() {
    }

    void initializeSubscribersAndPublishers();

    void rgbdCallback(const sensor_msgs::ImageConstPtr& depth_msg,
            const sensor_msgs::ImageConstPtr& rgb_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg);

    void publishOdometry(Eigen::Matrix4f& trans, Eigen::Matrix<float, 6, 6 > covMatrix,
            std::string& keyframe_frameid_str);

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    RGBDOdometryEngine(const RGBDOdometryEngine& yRef);
    RGBDOdometryEngine& operator=(const RGBDOdometryEngine& yRef);

    ros::NodeHandle nh;
    ros::NodeHandlePtr nodeptr;

    // variables held to process the current frame
    ros::Time frame_time;

    std::string prior_keyframe_frameid_str;

    // published odometry messages
    ros::Publisher pubXforms;
    ros::Publisher pubPose_w_cov;
    ros::Publisher pubOdom_w_cov;

    // subscribers to RGBD sensor data
    image_transport::SubscriberFilter sub_depthImage;
    image_transport::SubscriberFilter sub_rgbImage;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgbCameraInfo;

    ros::Time previous_time;
    double dt;
    std_msgs::Float64 dt_msg;
    ros::Publisher dt_pub;

};

#endif /* RGBD_ODOMETRY_H */
