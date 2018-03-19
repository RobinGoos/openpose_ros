#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"

#include <opencv2/core/core.hpp>

#include <openpose_ros_msgs/BoundingBox.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <openpose.h>
#include <gflags_options.h>
#include <openpose_ros_node.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPoseROSIO
    {
        private:
            ros::NodeHandle nh_;
            ros::Publisher openpose_human_list_pub_;
	        ros::Publisher openpose_human_count_pub_;
            ros::Publisher openpose_image_;
            ros::Subscriber param_sub_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            cv_bridge::CvImagePtr cv_img_ptr_;
            std_msgs::Header rgb_image_header_;

            OpenPose* openpose_;

        public:
            OpenPoseROSIO(OpenPose &openPose);

            ~OpenPoseROSIO(){}

            void processImage(const sensor_msgs::ImageConstPtr& msg);

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            std::shared_ptr<std::vector<op::Datum>> createDatum();

            void publishImageTopics(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

            void publishPersonCountTopic(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

            void parameterSubscriber();

            void paramCallback(const std_msgs::String::ConstPtr& msg);
    };
}

#endif
