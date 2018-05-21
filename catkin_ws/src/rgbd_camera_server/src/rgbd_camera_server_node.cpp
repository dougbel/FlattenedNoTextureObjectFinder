#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "vision_msgs/RecognizeFlattenedObjects.h"
#include "justina_tools/JustinaTools.h"

using namespace cv;
using namespace std;


ros::ServiceClient client;
vision_msgs::RecognizeFlattenedObjects srv;

sensor_msgs::PointCloud2 pc2_wrtRobot;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	pc2_wrtRobot = *cloud_msg;

}


bool robotRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)                                                                                          
{
	resp.point_cloud = pc2_wrtRobot;
	return true;                                                                                  
}


int main(int argc, char** argv) {


    ros::init(argc, argv, "rgbd_camera_server_node");
    

    ros::NodeHandle nodeHandle;
	
	ros::ServiceServer srvRgbdRobot;
	ros::Subscriber    subHSRpc2;
	ros::Rate          loopRate(50);
	
	cv::Mat bgrImg;
	cv::Mat xyzCloud;
    
    bool debug;

    
    debug = true;
	
	//topico para actualizar nube y que ofrece el servicio
	subHSRpc2    = nodeHandle.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, cloud_callback);
	srvRgbdRobot = nodeHandle.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", robotRgbd_callback);
	ROS_INFO_STREAM("Waiting for connections");
	
	ros::spin();

    return 0;
}
