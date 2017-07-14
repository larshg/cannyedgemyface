#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

class CannyEdgeNode
{
    ros::NodeHandle _nh;

    image_transport::ImageTransport _it;
    
    image_transport::Subscriber _sub;

    image_transport::Publisher _pub;

    cv::Mat src, src_gray;
    cv::Mat dst, detected_edges;

public:
    CannyEdgeNode() : _it(_nh)
    {
        ROS_INFO("Subscribing to /cv_camera/image_raw");
        _sub = _it.subscribe("/cv_camera/image_raw", 1, &CannyEdgeNode::imageCB,this);
        _pub = _it.advertise("/canny_edge/output_video",1);
    }



    void imageCB(const sensor_msgs::ImageConstPtr& msg)
    {
	cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImage cvCanny;

	try
	{
	   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	   ROS_ERROR("cv_bridge exception: %s", e.what());
	   return;
	}

        cv::cvtColor(cv_ptr->image,src_gray, CV_BGR2GRAY);

	/// Reduce noise with a kernel 3x3
	cv::blur( src_gray, detected_edges, cv::Size(3,3) );

	/// Canny detector
	cv::Canny( detected_edges, detected_edges, 15, 45, 3 );

	/// Using Canny's output as a mask, we display our result
	//dst = cv::Scalar::all(0);

	//src.copyTo( dst, detected_edges);

        cvCanny.header = msg->header;
	cvCanny.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        cvCanny.image = detected_edges;

        _pub.publish(cvCanny.toImageMsg());
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Canny_edge");

    CannyEdgeNode cannyEdgeNode;    

    ros::spin();

    return 0;
}
