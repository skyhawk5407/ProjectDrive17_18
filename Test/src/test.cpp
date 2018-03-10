// ZED includes
#include "ros/ros.h"
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

//may need to include?
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
void printHelp();

int main(int argc, char **argv)
{
  //set up ros
  ros::init(argc, argv, "testtest");
  ros::NodeHandle n;

  ros::Publisher translation_data = n.advertise<std_msg::String>("translation_data", 1000);
  ros::Rate loop_rate(10);

  //Set up the Zed Camera
  Camera zed;
  InitParameters init_params;
  // init_params.camera_resolution = RESOLUTION_HD72V0;//60fps
  init_params.camera_resolution = RESOLUTION_VGA;//60fps
  init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
  init_params.coordinate_units = UNIT_METER;

  //Open the camera
  ERROR_CODE err = zed.open(init_params);
  if(err != SUCCESS)
  {
    printf("%s\n", toString(err).c_str());
    zed.close();
    return 1;
  }

  printHelp();

  //Set Camera runtime parameters
  RuntimeParameters runtime_parameters;
  runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

  //Enable positional tracking
  TrackingParameters tracking_parameters;
  err = zed.enableTracking(tracking_parameters);
  if(err != SUCCESS)
  {
    printf("%s\n", toString(err).c_str());
    zed.close();
    return 2;
  }
  tracking_parameters.initial_world_transform = Transform::identity();
  tracking_parameters.enable_spatial_memory = true;

  //Prepare new image size to retrieve half-resolution images
  Resolution image_size = zed.getResolution();
  int new_width = image_size.width / 2;
  int new_height = image_size.height / 2;

  //Get left camera
  Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
  cv::Mat image_ocv = slMat2cvMat(image_zed);

  //variables
  Pose zed_pose;
  cv::Mat map = cv::Mat::zeros( 500, 500, CV_8UC1 );
  cv::Point prev;
  cv::Point curr;
  curr.x=250;
  curr.y=250;
  float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x *0.5f;
  std::ofstream outputFile;
  outputFile.open("data.txt");


  //Loop until 'q' is pressed.
  char key = ' ';
  while(key != 'q' && ros::ok())
  {
    if (zed.grab(runtime_parameters) == SUCCESS)
    {
      TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);
      zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
      
      if(state == TRACKING_STATE_OK)
      {
        prev = curr;
	
        //create the msg
        std_msgs::String msg;
        std::stringstream ss;

        ss<<std::setprecision(5)<<translation[0]<<","<<translation[1]<<","<<translation[2]<<", at "<<zed_pose.timestamp<<"\n";
        msg.data = ss.str();
        translation_data.publish(ss);
	
	ros::spinOnce();
	loop_rate.sleep();

        curr.x=250+10*translation[0];
        sl::float3 translation = zed_pose.getTranslation();
        printf("Translation: %.3f,%.3f,%.3f, %lu\n",translation[0],translation[1],translation[2],
		zed_pose.timestamp);
        outputFile<<std::setprecision(5)<<translation[0]<<","<<translation[1]<<","<<translation[2]<<","<<zed_pose.timestamp<<"\n";
        curr.x=250+10*translation[0];
        curr.y=250-10*translation[2];

        line(map, prev, curr, 127,2,8);
      }
      cv::imshow("Camera", image_ocv);
      cv::imshow("Map", map);

      key = cv::waitKey(10);
    }
  }
  outputFile.close();
  zed.close();
  return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input)
{
  //Mapping between MAT_TYPE and CV_TYPE
  int cv_type = -1;
  switch (input.getDataType())
  {
    case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
    case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
    case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
    case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
    case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
    case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
    case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
    case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
    default: break;
  }
  return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

void printHelp()
{
  std::cout << " Press 'q' to quit." << std::endl;
}
