#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "justina_tools/JustinaTools.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "obj_reco_norgbd_notexture_trainer/SegmenterTrainer.h"



using namespace cv;
using namespace std;


Mat imgSrc;
Mat element;


bool started=false;
int  xi, yi;

Scalar seedHSV;
Scalar seedHLS;

int structure_elem = 2;
int structure_size = 5;
int const max_elem = 2;
int const max_kernel_size = 21;


int minvalueH_HSV = 180;
int maxvalueH_HSV = 0;
int minvalueV_HSV = 255;
int maxvalueV_HSV = 0;
int minvalueS_HSV = 255;
int maxvalueS_HSV = 0;
int minvalueH_HLS = 180;
int maxvalueH_HLS = 0;
int minvalueL_HLS = 255;
int maxvalueL_HLS = 0;
int minvalueS_HLS = 255;
int maxvalueS_HLS = 0;
int minvalueB_BGR = 180;
int maxvalueB_BGR = 0;
int minvalueG_BGR = 255;
int maxvalueG_BGR = 0;
int minvalueR_BGR = 255;
int maxvalueR_BGR = 0;

char c;
bool seeTrackbars = false;
char lastChose = '1';

char str[400];


Scalar minScalar;
Scalar maxScalar;


void createElement( int, void* )
{
    int erosion_type;
    if( structure_elem == 0 ){ erosion_type = MORPH_RECT; }
    else if( structure_elem == 1 ){ erosion_type = MORPH_CROSS; }
    else if( structure_elem == 2) { erosion_type = MORPH_ELLIPSE; }

    Size size   = Size( 2*structure_size + 1, 2*structure_size+1 );
    Point point = Point( structure_size, structure_size );

    element     = getStructuringElement( erosion_type, size, point);
}

string elementToString(){
    stringstream ss;
    string erosion_type;
    if( structure_elem == 0 ){ erosion_type = "MORPH_RECT"; }
    else if( structure_elem == 1 ){ erosion_type = "MORPH_CROSS"; }
    else if( structure_elem == 2) { erosion_type = "MORPH_ELLIPSE"; }

    ss <<"Mat element = getStructuringElement( "<<erosion_type <<", Size("<< 2*structure_size + 1<<","<< 2*structure_size+1 << "), Point( "<<structure_size<<","<< structure_size <<"));";
    return ss.str();

}


void onMouse(int event, int x, int y, int flags, void *params)
{
    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            Mat   hsv;
            Vec3b hsv_vec;

            cvtColor(imgSrc, hsv, COLOR_BGR2HSV);
            hsv_vec = hsv.at<Vec3b>(y,x);
            seedHSV = Scalar( hsv_vec.val[0], hsv_vec.val[1], hsv_vec.val[2] );

            if(maxvalueH_HSV < hsv_vec.val[0] )
                    maxvalueH_HSV = hsv_vec.val[0];
            if(minvalueH_HSV > hsv_vec.val[0] )
                    minvalueH_HSV = hsv_vec.val[0];
            if(maxvalueS_HSV < hsv_vec.val[1] )
                    maxvalueS_HSV = hsv_vec.val[1];
            if(minvalueS_HSV > hsv_vec.val[1] )
                    minvalueS_HSV = hsv_vec.val[1];
            if(maxvalueV_HSV < hsv_vec.val[2] )
                    maxvalueV_HSV = hsv_vec.val[2];
            if(minvalueV_HSV > hsv_vec.val[2] )
                    minvalueV_HSV = hsv_vec.val[2];


            Mat   hls;
            Vec3b hls_vec;

            cvtColor(imgSrc, hls, COLOR_BGR2HLS);
            hls_vec = hls.at<Vec3b>(y,x);
            seedHLS = Scalar( hls_vec.val[0], hls_vec.val[1], hls_vec.val[2] );

            if(maxvalueH_HLS < hls_vec.val[0] )
                    maxvalueH_HLS = hls_vec.val[0];
            if(minvalueH_HLS > hls_vec.val[0] )
                    minvalueH_HLS = hls_vec.val[0];
            if(maxvalueL_HLS < hls_vec.val[1] )
                    maxvalueL_HLS = hls_vec.val[1];
            if(minvalueL_HLS > hls_vec.val[1] )
                    minvalueL_HLS = hls_vec.val[1];
            if(maxvalueS_HLS < hls_vec.val[2] )
                    maxvalueS_HLS = hls_vec.val[2];
            if(minvalueS_HLS > hls_vec.val[2] )
                    minvalueS_HLS = hls_vec.val[2];

            Mat   bgr;
            Vec3b bgr_vec;

            bgr     = imgSrc;
            bgr_vec = bgr.at<Vec3b>(y,x);
            seedHLS = Scalar( bgr_vec.val[0], bgr_vec.val[1], bgr_vec.val[2] );

            if(maxvalueB_BGR < bgr_vec.val[0] )
                    maxvalueB_BGR = bgr_vec.val[0];
            if(minvalueB_BGR > bgr_vec.val[0] )
                    minvalueB_BGR = bgr_vec.val[0];
            if(maxvalueG_BGR < bgr_vec.val[1] )
                    maxvalueG_BGR = bgr_vec.val[1];
            if(minvalueG_BGR > bgr_vec.val[1] )
                    minvalueG_BGR = bgr_vec.val[1];
            if(maxvalueR_BGR < bgr_vec.val[2] )
                    maxvalueR_BGR = bgr_vec.val[2];
            if(minvalueR_BGR > bgr_vec.val[2] )
                    minvalueR_BGR = bgr_vec.val[2];

            started = true;
        break;
    }
}

void updateTrackBars(){

    setTrackbarPos( "min H", "Segmentation_HSV", minvalueH_HSV );
    setTrackbarPos( "max H", "Segmentation_HSV", maxvalueH_HSV );
    setTrackbarPos( "min S", "Segmentation_HSV", minvalueS_HSV );
    setTrackbarPos( "max S", "Segmentation_HSV", maxvalueS_HSV );
    setTrackbarPos( "min V", "Segmentation_HSV", minvalueV_HSV );
    setTrackbarPos( "max V", "Segmentation_HSV", maxvalueV_HSV );

    setTrackbarPos( "min H", "Segmentation_HLS", minvalueH_HLS );
    setTrackbarPos( "max H", "Segmentation_HLS", maxvalueH_HLS );
    setTrackbarPos( "min L", "Segmentation_HLS", minvalueL_HLS );
    setTrackbarPos( "max L", "Segmentation_HLS", maxvalueL_HLS );
    setTrackbarPos( "min S", "Segmentation_HLS", minvalueS_HLS );
    setTrackbarPos( "max S", "Segmentation_HLS", maxvalueS_HLS );

    setTrackbarPos( "min B", "Segmentation_BGR", minvalueB_BGR );
    setTrackbarPos( "max B", "Segmentation_BGR", maxvalueB_BGR );
    setTrackbarPos( "min G", "Segmentation_BGR", minvalueG_BGR );
    setTrackbarPos( "max G", "Segmentation_BGR", maxvalueG_BGR );
    setTrackbarPos( "min R", "Segmentation_BGR", minvalueR_BGR );
    setTrackbarPos( "max R", "Segmentation_BGR", maxvalueR_BGR );
}

void createTrackBars()
{

    cvNamedWindow( "Segmentation_HSV", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Segmentation_HLS", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "Segmentation_BGR", CV_WINDOW_AUTOSIZE );

    createTrackbar( "min H", "Segmentation_HSV", &minvalueH_HSV, 180 );
    createTrackbar( "max H", "Segmentation_HSV", &maxvalueH_HSV, 180 );
    createTrackbar( "min S", "Segmentation_HSV", &minvalueS_HSV, 255 );
    createTrackbar( "max S", "Segmentation_HSV", &maxvalueS_HSV, 255 );
    createTrackbar( "min V", "Segmentation_HSV", &minvalueV_HSV, 255 );
    createTrackbar( "max V", "Segmentation_HSV", &maxvalueV_HSV, 255 );

    createTrackbar( "min H", "Segmentation_HLS", &minvalueH_HLS, 180 );
    createTrackbar( "max H", "Segmentation_HLS", &maxvalueH_HLS, 180 );
    createTrackbar( "min L", "Segmentation_HLS", &minvalueL_HLS, 255 );
    createTrackbar( "max L", "Segmentation_HLS", &maxvalueL_HLS, 255 );
    createTrackbar( "min S", "Segmentation_HLS", &minvalueS_HLS, 255 );
    createTrackbar( "max S", "Segmentation_HLS", &maxvalueS_HLS, 255 );

    createTrackbar( "min B", "Segmentation_BGR", &minvalueB_BGR, 180 );
    createTrackbar( "max B", "Segmentation_BGR", &maxvalueB_BGR, 180 );
    createTrackbar( "min G", "Segmentation_BGR", &minvalueG_BGR, 255 );
    createTrackbar( "max G", "Segmentation_BGR", &maxvalueG_BGR, 255 );
    createTrackbar( "min R", "Segmentation_BGR", &minvalueR_BGR, 255 );
    createTrackbar( "max R", "Segmentation_BGR", &maxvalueR_BGR, 255 );
}

int rectBoundColorMaxArea(Mat mask, Mat& output, string nameRect){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a=contourArea( contours[i],false);
        if(a>largest_area){
            largest_area=a;
            // Store the index of largest contour
            largest_contour_index=i;
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }
    }

    rectangle(output, bounding_rect,  Scalar(0,255,0),2, 8,0);

    putText(output, nameRect, Point2f(bounding_rect.x,bounding_rect.y), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));
    return largest_area;
}



void rgbdCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){



		 cv::Mat xyzCloud;
		 JustinaTools::PointCloud2Msg_ToCvMat(cloud_msg, imgSrc, xyzCloud);
		 //imshow("Input", bgrImg);
		 //waitKey(15);

       Mat bgr;
      // imgSrc.copyTo(bgr);
        blur( imgSrc, bgr, Size( 3, 3 ), Point(-1,-1) );

       minScalar = Scalar(minvalueH_HSV,minvalueS_HSV,minvalueV_HSV);
       maxScalar = Scalar(maxvalueH_HSV, maxvalueS_HSV, maxvalueV_HSV);
       Mat resultHSV = SegmenterTrainer::colorSegmentHSV( bgr, element, minScalar, maxScalar);

       minScalar = Scalar(minvalueH_HLS, minvalueL_HLS, minvalueS_HLS);
       maxScalar = Scalar(maxvalueH_HLS, maxvalueL_HLS, maxvalueS_HLS);
       Mat resultHLS = SegmenterTrainer::colorSegmentHLS( bgr, element,minScalar, maxScalar);

       Mat resultH   = SegmenterTrainer::colorSegmentH( bgr, element, minvalueH_HLS, maxvalueH_HLS);

       minScalar = Scalar(minvalueB_BGR, minvalueG_BGR, minvalueR_BGR);
       maxScalar = Scalar(maxvalueB_BGR, maxvalueG_BGR, maxvalueR_BGR);
       Mat resultBGR = SegmenterTrainer::colorSegmentBGR( bgr, element, minScalar, maxScalar);

       int largest_area;
       switch ( lastChose ) {
           case '1':
               largest_area = rectBoundColorMaxArea(resultHSV,bgr,"HSV");
           break;
           case '2':
               largest_area = rectBoundColorMaxArea(resultHLS,bgr,"HLS");
               break;
           case '3':
               largest_area = rectBoundColorMaxArea(resultBGR,bgr,"BGR");
               break;
           case '4':
               largest_area = rectBoundColorMaxArea(resultH,bgr,"H");
               break;
           default:
               largest_area = rectBoundColorMaxArea(resultHSV,bgr,"HSV");

       }



       sprintf(str,"Max area: %i",largest_area);
       putText(bgr, str, Point2f(10,10), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));
       putText(bgr, "t : see/dissapear trackbars", Point2f(10,30), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));
       putText(bgr, "p : print trained values", Point2f(10,40), FONT_HERSHEY_PLAIN, 0.9,  Scalar(017,070,244,255));


       cvtColor(resultHSV, resultHSV,CV_GRAY2BGR);
       putText(resultHSV, "HSV", Point2f(20,20), FONT_HERSHEY_PLAIN, 1.5,  Scalar(017,070,244,255),2);

       cvtColor(resultHLS, resultHLS,CV_GRAY2BGR);
       putText(resultHLS, "HLS", Point2f(20,20), FONT_HERSHEY_PLAIN, 1.5,  Scalar(017,070,244,255),2);

       cvtColor(resultH, resultH,CV_GRAY2BGR);
       putText(resultH, "H", Point2f(20,20), FONT_HERSHEY_PLAIN, 1.5,  Scalar(017,070,244,255),2);

       cvtColor(resultBGR, resultBGR,CV_GRAY2BGR);
       putText(resultBGR, "BGR", Point2f(20,20), FONT_HERSHEY_PLAIN, 1.5,  Scalar(017,070,244,255),2);

       Mat output;
       Mat h1,h2;
       hconcat(resultHSV, resultHLS, h1);
       hconcat(resultBGR, resultH, h2);
       vconcat(h1, h2, output);

       resize(output, output, cv::Size(), 0.5, 0.5);

       imshow("Input", bgr);
       imshow("Output", output);

       if(seeTrackbars){
           imshow("Segmentation_H", resultH);
           imshow("Segmentation_HSV", resultHSV);
           imshow("Segmentation_HLS", resultHLS);
           imshow("Segmentation_BGR", resultBGR);
           updateTrackBars();

       }


       c=cvWaitKey(5);

       if(c != -1) lastChose = c;

       if(c == 't'){
           seeTrackbars = !seeTrackbars;
           if(seeTrackbars){
               createTrackBars();
           }
           else
               destroyAllWindows();
       }

       if(c=='p'){
           cout << elementToString() << endl << endl;
           sprintf(str,"HSV: \nminScalar = Scalar(%i, %i, %i);\nmaxScalar = Scalar(%i, %i, %i);",minvalueH_HSV,minvalueS_HSV,minvalueV_HSV,maxvalueH_HSV, maxvalueS_HSV, maxvalueV_HSV);
           cout << str <<endl<<endl;
           sprintf(str,"HLS: \nminScalar = Scalar(%i, %i, %i);\nmaxScalar = Scalar(%i, %i, %i);",minvalueH_HLS, minvalueL_HLS, minvalueS_HLS, maxvalueH_HLS, maxvalueL_HLS, maxvalueS_HLS);
           cout << str <<endl<<endl;
           sprintf(str,"BGR: \nminScalar = Scalar(%i, %i, %i);\nmaxScalar = Scalar(%i, %i, %i);",minvalueB_BGR, minvalueG_BGR, minvalueR_BGR, maxvalueB_BGR, maxvalueG_BGR, maxvalueR_BGR);
           cout << str <<endl<<endl;
           sprintf(str,"H: \nminH = %i;\nmaxH = %i;", minvalueH_HLS, maxvalueH_HLS);
           cout << str <<endl<<endl;
       }

}


int main(int argc, char** argv) {

	createElement(0,0);

	cvNamedWindow( "Input", CV_WINDOW_AUTOSIZE );
	cvSetMouseCallback("Input", onMouse, 0);
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Input",&structure_elem, max_elem, createElement);
	createTrackbar( "Kernel size:\n 2n +1", "Input",&structure_size, max_kernel_size, createElement);


	ros::init(argc, argv, "obj_reco_norgbd_notexture");
	ros::ServiceClient cltRgbdRobot;
	ros::NodeHandle nodeHandle("~");
	ros::Rate loopRate(30);

	cv::Mat bgrImg;
	cv::Mat xyzCloud;

	ROS_INFO_STREAM("Todo bien, gracias");

	ros::Subscriber subscriber = nodeHandle.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot",30,rgbdCallback);
  ros::spin();

	/*cltRgbdRobot = nodeHandle.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/rgbd_wrt_kinect");


	while (ros::ok()) {
		point_cloud_manager::GetRgbd srv;
		if(cltRgbdRobot.call(srv))
		{
			JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImg, xyzCloud);

			imshow("Depth", xyzCloud);
			imshow("RGB", bgrImg);

		}
		else
					std::cout << "Cannot get point cloud" << std::endl;

		ros::spinOnce();
		loopRate.sleep();
	}*/

	return 0;
}
