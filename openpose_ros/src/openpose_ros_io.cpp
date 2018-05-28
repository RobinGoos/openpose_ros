#include <openpose_ros_io.h>

using namespace openpose_ros;

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): it_(nh_)
{
    std::string image_topic;
    std::string paramTopic;
    std::string output_topic;
    std::string output_person_count;
    std::string output_topic_video;


    nh_.param("/openpose_ros_node/image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("/openpose_ros_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("/openpose_ros_node/output_topic_2", output_person_count, std::string("/openpose_ros/human_count"));
    nh_.param("/openpose_ros_node/output_topic_video", output_topic_video, std::string("/openpose_ros/image"));


    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this);
    param_sub_ = nh_.subscribe(paramTopic, 100, &OpenPoseROSIO::paramCallback, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    openpose_human_count_pub_ = nh_.advertise<std_msgs::String>(output_person_count, 10);
    openpose_image_ = nh_.advertise<sensor_msgs::Image>(output_topic_video, 15);


    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);

    std::shared_ptr<std::vector<op::Datum>> datumProcessed;
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {
        publishImageTopics(datumProcessed);
        publishPersonCountTopic(datumProcessed);
    }
    else
    {
        op::log("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }
}

void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgb_image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

std::shared_ptr<std::vector<op::Datum>> OpenPoseROSIO::createDatum()
{
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else
    {
        auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        datum.cvInputData = cv_img_ptr_->image;

        return datumsPtr;
    }
}

// Takes the openpose output image and converts it to a ROS topic. /openpose_ros/image
void OpenPoseROSIO::publishImageTopics(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr){
    auto outputIm = datumsPtr->at(0).cvOutputData;
    auto outputHeatmap = datumsPtr->at(0).poseHeatMaps;
    op::log(outputHeatmap);
    cv::putText(outputIm, "Person count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0)),
                cv::Point(25,150),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255),1);


    sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputIm).toImageMsg();
    openpose_image_.publish(img);
}

// Check size of keypoint array and outputs it as Person Count over /openpose_ros/human_count topics
void OpenPoseROSIO::publishPersonCountTopic(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    std_msgs::String msg;
    msg.data = "Person Count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0));

    openpose_human_count_pub_.publish(msg);
}

void OpenPoseROSIO::paramCallback(const std_msgs::String::ConstPtr& msg){
    openpose_->stop();
    openPoseROS(std::stoi(msg->data.c_str()));
}
