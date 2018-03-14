#include <openpose_ros_io.h>

using namespace openpose_ros;

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string output_topic;
    std::string output_topic_2;
    std::string output_topic_video;


    nh_.param("/openpose_ros_node/image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("/openpose_ros_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("/openpose_ros_node/output_topic_2", output_topic_2, std::string("/openpose_ros/human_count"));
    nh_.param("/openpose_ros_node/output_topic_video", output_topic_video, std::string("/openpose_ros/image"));

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    openpose_human_count_pub_ = nh_.advertise<std_msgs::String>(output_topic_2, 10);
    openpose_image_ = nh_.advertise<sensor_msgs::Image>(output_topic_video, 15);

    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);
    
    // Pop frame
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
    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        // Fill datum
        datum.cvInputData = cv_img_ptr_->image;

        return datumsPtr;
    }
}

bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

void OpenPoseROSIO::publishImageTopics(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr){
    auto outputIm = datumsPtr->at(0).cvOutputData;
    cv::putText(outputIm, "Person count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0)),
                cv::Point(25,25),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255),1);


    sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputIm).toImageMsg();
    openpose_image_.publish(img);
}

void OpenPoseROSIO::publishPersonCountTopic(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    std_msgs::String msg;
    msg.data = "Person Count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0));

    openpose_human_count_pub_.publish(msg);
}

