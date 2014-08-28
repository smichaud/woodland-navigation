#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageSaver
{
private:
    ros::NodeHandle nodeHandle;
    const std::string subscribeTopic;
    const std::string filename;

    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;

public:
    ImageSaver(ros::NodeHandle& nodeHandle, std::string filename):
        nodeHandle(nodeHandle),
        subscribeTopic("/axis/image_raw"),
        filename(filename),
        imageTransport(nodeHandle) {
        imageSubscriber = imageTransport.subscribe(
                    subscribeTopic, 1,
                    &ImageSaver::gotImage, this,
                    image_transport::TransportHints("compressed"));
    }

    void gotImage(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cvImageptr;
        try {
            cvImageptr = cv_bridge::toCvCopy(msg,
                                         sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("A cv_bridge exception occured: %s", e.what());
        }

        cv::imwrite(filename, cvImageptr->image);
        ROS_INFO("Image saved to: %s", filename.c_str());

        ros::shutdown();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_extractor");

    ros::NodeHandle nodeHandle("~");
    std::string fileName;
    nodeHandle.param<std::string>("filename", fileName,
                                  "./image_extracted.jpg");

    ImageSaver imageSaver(nodeHandle, fileName);

    ros::spin();
    return 0;
}
