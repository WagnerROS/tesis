#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <geometry_msgs/Point.h>

class PeopleDetection
{
public:
    PeopleDetection()
    {
        // Initialize ROS
        nh_ = ros::NodeHandle("~");

        // Subscribe to RGB image topic
        image_sub_ = nh_.subscribe("/processed/rgb/image_raw", 1, &PeopleDetection::imageCallback, this);

        // Publisher for detection coordinates
        detection_pub_ = nh_.advertise<geometry_msgs::Point>("detection_coordinates", 1);

        // Load the pre-trained HOG descriptor and SVM for person detection
        hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& input)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect people in the image
        std::vector<cv::Rect> detections;
        hog_.detectMultiScale(cv_ptr->image, detections);

        for (size_t i = 0; i < detections.size(); i++)
        {
            cv::rectangle(cv_ptr->image, detections[i], cv::Scalar(0, 255, 0), 2);

            // Calculate the center of the detection
            int center_x = detections[i].x + detections[i].width / 2;
            int center_y = detections[i].y + detections[i].height / 2;

            // Publish the coordinates
            geometry_msgs::Point center_coords;
            center_coords.x = center_x;
            center_coords.y = center_y;
            center_coords.z = 0.0;
            detection_pub_.publish(center_coords);

            ROS_INFO("Person detected at center coordinates: (%d, %d)", center_x, center_y); // Added for debugging
        }

        // Display the result
        cv::imshow("People Detection", cv_ptr->image);
        cv::waitKey(3);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher detection_pub_;
    cv::HOGDescriptor hog_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "people_detection_node");
    PeopleDetection people_detection;
    ros::spin();
    return 0;
}
