/*
 * File:   Feature3DEngine.cpp
 * Author: arwillis
 *
 * Created on August 18, 2015, 10:35 AM
 */
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <random>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL includes
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// TF includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Includes for this Library
#include <rgbd_odometry/rgbd_odometry_ros.h>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>

void toString(pcl::PointXYZRGB& ptrgb) {
    ROS_INFO("x,y,z=(%f,%f,%f) r,g,b=(%d,%d,%d)",
            ptrgb.x, ptrgb.y, ptrgb.z,
            ptrgb.rgba >> 24, (ptrgb.rgba & 0x00FFFFFF) >> 16, (ptrgb.rgba & 0x0000FFFF) >> 8);
}

namespace Eigen {

    void toString(std::string name, Eigen::MatrixXf mat) {
        static std::string sep = "\n----------------------------------------\n";
        static int StreamPrecision = 4;
        static Eigen::IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        std::cout << sep << name << " = " << mat.format(OctaveFmt) << ";" << sep;
    }
}

void RGBDOdometryEngine::rgbdCallback(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::ImageConstPtr& rgb_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg) {

    ros::Time timestamp = depth_msg->header.stamp;
    dt_msg.data = (timestamp - previous_time).toSec();

    if(dt_msg.data > 0.036 || dt_msg.data < 0.03){
//      ROS_WARN_STREAM("The image is rejected! Dt was "<< dt_msg.data);
      previous_time = timestamp;
      // return;
    }
    dt_pub.publish(dt_msg);

    previous_time = timestamp;
    static int frame_id = 0;
    if (VERBOSE) {
        ROS_DEBUG("Heard rgbd image.");
    }
    frame_time = depth_msg->header.stamp;
    std::string keyframe_frameid_str("frame_");
    keyframe_frameid_str.append(stdpatch::to_string(frame_id++));

    cv_bridge::CvImageConstPtr rgb_img_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr depth_img_ptr;
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        depth_img_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_img_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height) {
        ROS_ERROR("Depth and RGB image dimensions don't match depth=( %dx%d ) rgb=( %dx%d )!",
                depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
    }

    if (!hasRGBCameraIntrinsics()) {
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(info_msg);
        cv::Mat Kdouble(model.intrinsicMatrix());
        cv::Mat Kfloat;
        Kdouble.convertTo(Kfloat, CV_32FC1);
        setRGBCameraIntrinsics(Kfloat);
    }

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

    Eigen::Matrix<float, 6, 6> covMatrix = Eigen::Matrix<float, 6, 6>::Zero();
    bool odomEstimatorSuccess = false;

    if (!DIRECT_ODOM) {
        cv::UMat depthimg = depth_img_ptr->image.getUMat(cv::ACCESS_READ);
        cv::UMat frame = rgb_img_ptr->image.getUMat(cv::ACCESS_READ);
        odomEstimatorSuccess = computeRelativePose(frame, depthimg, trans, covMatrix);
    } else {
        static cv_bridge::CvImageConstPtr prev_rgb_img_ptr, prev_depth_img_ptr;
        if (prev_rgb_img_ptr) {
            odomEstimatorSuccess = computeRelativePoseDirectMultiScale(
                    prev_rgb_img_ptr->image, prev_depth_img_ptr->image, // warp image
                    rgb_img_ptr->image, depth_img_ptr->image, // template image
                    trans, covMatrix,
                    100, 3, 1);
        }
        prev_rgb_img_ptr = rgb_img_ptr;
        prev_depth_img_ptr = depth_img_ptr;
    }

    if (odomEstimatorSuccess) {
        publishOdometry(trans, covMatrix, keyframe_frameid_str);

    }
    prior_keyframe_frameid_str = keyframe_frameid_str;
}

void RGBDOdometryEngine::publishOdometry(Eigen::Matrix4f& trans, Eigen::Matrix<float, 6, 6 > covMatrix,
        std::string& keyframe_frameid_str) {
    Eigen::Quaternionf quat(trans.block<3, 3>(0, 0));
    Eigen::Vector3f translation(trans.block<3, 1>(0, 3));

    tf::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
    tf::Transform xform(tf_quat,
            tf::Vector3(translation[0], translation[1], translation[2]));
    tf::StampedTransform xformStamped(xform, frame_time, keyframe_frameid_str, keyframe_frameid_str);
    geometry_msgs::TransformStamped gxform;
    tf::transformStampedTFToMsg(xformStamped, gxform);
    gxform.header.frame_id = keyframe_frameid_str;
    geometry_msgs::PoseWithCovarianceStamped odom_w_cov_msg;
    odom_w_cov_msg.pose.pose.orientation = gxform.transform.rotation;
    odom_w_cov_msg.pose.pose.position.x = gxform.transform.translation.x;
    odom_w_cov_msg.pose.pose.position.y = gxform.transform.translation.y;
    odom_w_cov_msg.pose.pose.position.z = gxform.transform.translation.z;
    int offset;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            offset = col * 6 + row;
            odom_w_cov_msg.pose.covariance[offset] = covMatrix(row, col);
        }
    }
    odom_w_cov_msg.header.stamp = frame_time;
    odom_w_cov_msg.header.frame_id = keyframe_frameid_str;
    pubOdom_w_cov.publish(odom_w_cov_msg);

}

void RGBDOdometryEngine::initializeSubscribersAndPublishers() {
    int queue_size = 10;
    //ros::NodeHandlePtr nodeptr(new ros::NodeHandle);
    //nodeptr(new ros::NodeHandle)
    image_transport::ImageTransport it_depth(*nodeptr);
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";
    // depth image can use different transport.(e.g. compressedDepth)
    image_transport::TransportHints depth_hints("raw", ros::TransportHints(),
            *nodeptr, depth_image_transport_param);
    //    image_transport::Subscriber sub_depthImage = it_depth.subscribe("depth/image_raw", 1, depth_hints);
    //image_transport::SubscriberFilter sub_depthImage;
    sub_depthImage.subscribe(it_depth, "depth_registered/input_image", 1, depth_hints);

    //message_filters::Subscriber<sensor_msgs::CameraInfo>
    //        sub_rgbImage(*nodeptr, "rgb/image_raw", 1);
    image_transport::ImageTransport it_rgb(*nodeptr);
    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints("raw", ros::TransportHints(), *nodeptr);
    //image_transport::SubscriberFilter sub_rgbImage;
    sub_rgbImage.subscribe(it_rgb, "rgb/input_image", 1, hints);

    //    message_filters::Subscriber<sensor_msgs::CameraInfo>
    //            sub_rgbCameraInfo;
    sub_rgbCameraInfo.subscribe(*nodeptr, "rgb/camera_info", 1);

    // option 1
    //    message_filters::TimeSynchronizer
    //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
    //            syncTime(sub_depthImage, sub_rgbImage, sub_rgbCameraInfo, queue_size);
    //    syncTime.registerCallback(boost::bind(&Feature3DEngine::rgbdImageCallback,
    //            engineptr, _1, _2, _3));
    // option 2
    //    typedef message_filters::sync_policies::ExactTime
    //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(queue_size)
    //    message_filters::Synchronizer<MyExactSyncPolicy> syncExact(MyExactSyncPolicy(queue_size),
    //            sub_depthImage, sub_rgbImage, sub_rgbCameraInfo);
    // option 3
    typedef message_filters::sync_policies::ApproximateTime
            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproximateSyncPolicy;
    static message_filters::Synchronizer<MyApproximateSyncPolicy> syncApprox(MyApproximateSyncPolicy(queue_size),
            sub_depthImage, sub_rgbImage, sub_rgbCameraInfo);
    //syncApprox.registerCallback(boost::bind(&RGBDOdometryEngine::rgbdImageCallback,
    //        this, _1, _2, _3));
    syncApprox.registerCallback(boost::bind(&RGBDOdometryEngine::rgbdCallback,
            this, _1, _2, _3));

    pubXforms = nodeptr->advertise<geometry_msgs::TransformStamped>("relative_xform", 1000);
    pubPose_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_w_cov", 1000);
    pubOdom_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_w_cov", 1000);
    dt_pub = nodeptr->advertise<std_msgs::Float64>("dt_images", 1);
}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "uncc_rgbd_odom");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    RGBDOdometryEngine engine;

    // %Tag(SUBSCRIBER)%
    engine.initializeSubscribersAndPublishers();

    engine.getImageFunctionProvider()->computeFilterBank();
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    engine.getImageFunctionProvider()->freeFilterBank();
    return 0;
}
