#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "vision_msgs/RecognizeFlattenedObjects.h"
#include "justina_tools/JustinaTools.h"

using namespace cv;
using namespace std;


ros::ServiceClient client;
vision_msgs::RecognizeFlattenedObjects srv;

void call_service_rgbd_camera(ros::NodeHandle nodeHandle){
	
	cv::Mat imgSrc;
	cv::Mat xyzCloud;
	
	
	ros::Rate loopRate(50);
	
	ros::service::waitForService("/hardware/point_cloud_man/get_rgbd_wrt_robot", -1);
	
	ros::ServiceClient  clt_RgbdRobot;
	clt_RgbdRobot = nodeHandle.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
	
	int i=0;
	
	
	while (ros::ok()) {
		cv::Mat bgrImg;
		cv::Mat xyzCloud;
		
		
		point_cloud_manager::GetRgbd srv;
		
		
		while(!clt_RgbdRobot.call(srv))
		{
			ROS_INFO_STREAM("Cannot get point cloud");
		}
		
		JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImg, xyzCloud);
		stringstream ss;
		ss << i++;
		putText(bgrImg, ss.str(), Point2f(10, 10), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));
		
		imshow("rgb", bgrImg);
		waitKey(30);
		
		ros::spinOnce();
		loopRate.sleep();
		
		}	
}

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


    ros::init(argc, argv, "obj_reco_flattened_client_node");
    

    ros::NodeHandle nodeHandle;
	
	
	
	cv::Mat bgrImg;
	cv::Mat xyzCloud;
    
    

	//call_service_rgbd_camera(ros::NodeHandle nodeHandle)
	
	
	ros::Rate loopRate(50);
	
	ros::service::waitForService("/vision/obj_reco/flattened_object", 10000);
	
	ros::ServiceClient  client;
	client = nodeHandle.serviceClient<vision_msgs::RecognizeFlattenedObjects>("/vision/obj_reco/flattened_object");

	vision_msgs::RecognizeFlattenedObjects srv;
	
	
	while(!client.call(srv))
	{
		ROS_INFO_STREAM("It is not posibble stablish a connection with the segmentator node");
	}
	
	ROS_INFO_STREAM(srv.response.recog_objects.objectList.size());
	ROS_INFO_STREAM(srv.response.recog_objects.objectList[0].id);
	
	cv_bridge::CvImagePtr cvImageMask;
	cvImageMask = cv_bridge::toCvCopy(srv.response.recog_objects.maskOfObjects,
								  sensor_msgs::image_encodings::MONO8);
	Mat maskOfObjects = cvImageMask->image;
	
	cv_bridge::CvImagePtr cvImageFounds;
	cvImageFounds = cv_bridge::toCvCopy(srv.response.recog_objects.imgOutput,
								  sensor_msgs::image_encodings::BGR8);
	Mat objectsFound = cvImageFounds->image;
	
	
	imshow("maskOfObjects", maskOfObjects);
	imshow("objectsFound", objectsFound);
	waitKey(0);
	

	return 0;
}
