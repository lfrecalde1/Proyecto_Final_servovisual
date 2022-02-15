#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <typeinfo>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cv_basics/image_data.h>
// Read Camera coeficients distortion
cv::Mat mtx_l, dist_l;
cv::Mat mtx_r, dist_r;

// Undistor matrices
cv::Mat img_r;
cv::Mat img_l;


// name images
std::string left = "left";
std::string right = "right";

// Aruco Dictionay
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

cv::FileStorage fsr("/home/fer/catkin_ws/src/cv_basics/src/right_parameters.yml", cv::FileStorage::READ); 
cv::FileStorage fsl("/home/fer/catkin_ws/src/cv_basics/src/left_parameters.yml", cv::FileStorage::READ); 

// UNDISTOR IMAGE
cv::Mat correction(cv::Mat, cv::Mat, cv::Mat);
cv::Mat aruco_process(cv::Mat);
std::vector<std::vector<cv::Point2f>> aruco_corners(cv::Mat);
Eigen::MatrixXd better_points(std::vector<std::vector<cv::Point2f>>);
Eigen::MatrixXd point_3d(std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>, cv::Mat, cv::Mat, double);
Eigen::MatrixXd center_camera(Eigen::MatrixXd, cv::Mat);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;

  // Variables for shape
  int h;
  int w;
  int half_w;
  // Matrices cameras
  fsr["K"] >> mtx_r;
  fsl["K"] >> mtx_l;
  fsr["D"] >> dist_r;
  fsl["D"] >> dist_l;

  // distence between cameras
  double B = 0.2;



   
  try
  { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    cv::Mat current_frame = cv_ptr->image;
    cv::Mat aux_l;
    cv::Mat aux_r;

    // corners aux
    std::vector<std::vector<cv::Point2f>> aux_l_corner;
    std::vector<std::vector<cv::Point2f>> aux_r_corner;

    // corner_modificadas sistema eigen
    Eigen::MatrixXd system_coordinates;



    // get image shape
    h = current_frame.size[0];
    w = current_frame.size[1];
    half_w = (int)(w/2);
    cv::Rect roil(0, 0, half_w, h);
    cv::Rect roir(half_w, 0, half_w, h);

    img_l = correction(current_frame(roil),mtx_l, dist_l);
    img_r = correction(current_frame(roir),mtx_r, dist_r);

    aux_l = aruco_process(img_l);
    aux_r = aruco_process(img_r);

    aux_l_corner = aruco_corners(img_l);
    aux_r_corner = aruco_corners(img_r);

    if (!aux_l.empty()){

        //finales_l = better_points(aux_l_corner, aux_l);
        system_coordinates = point_3d(aux_l_corner, aux_r_corner, mtx_l, mtx_r, B);
        std::cout <<system_coordinates<<std::endl;
        std::cout << "...............................\n" << std::endl;
        cv::imshow("Left", aux_l);
        cv::waitKey(3);
        //int value = 2;
        //pixel_message.u1 = value;
        //pixel_pub.publish(pixel_message);
    }

    

    // Display frame for 30 milliseconds
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "Opencv_process_Mavic_2_PRO");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("/Mavic_2_PRO/camera_lr", 1, imageCallback);

  //ros::Publisher pixel_pub = nh.advertise<cv_basics::image_data>("Mavic_2_PRO/Pixels_data", 10);
  //cv_basics::image_data pixel_message;

   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  cv::destroyWindow("view");
}

cv::Mat correction(cv::Mat frame, cv::Mat mtx, cv::Mat dst)
{
    int h_frame;
    int w_frame;
    cv::Mat new_camera;
    cv::Mat frame_final;

    h_frame = frame.size[0];
    w_frame = frame.size[1];

    //new_camera = cv::getOptimalNewCameraMatrix(mtx, dst, {w_frame, h_frame}, 1);
    cv::undistort(frame, frame_final, mtx, dst);
    return frame_final;

}

cv::Mat aruco_process(cv::Mat frame){
    // vector of id identification
    std::vector<int> ids;

    // Vector of corners
    std::vector<std::vector<cv::Point2f> > corners;

    // Aux image to process the points
    cv::Mat img_l_gray;
    cv::Mat img_l_copy;

    // Transform RGB TO GRAY
    cv::cvtColor(frame, img_l_gray, cv::COLOR_BGR2GRAY);

    // Aruco detection
    cv::aruco::detectMarkers(img_l_gray, dictionary, corners, ids);

    if (ids.size()>0)
    {
        img_l_copy = img_l_gray.clone();
        cv::aruco::drawDetectedMarkers(img_l_copy, corners, ids);
    }
    return img_l_copy;

}

std::vector<std::vector<cv::Point2f>> aruco_corners(cv::Mat frame){
    // vector of id identification
    std::vector<int> ids;

    // Vector of corners
    std::vector<std::vector<cv::Point2f>> corners;

    // Aux image to process the points
    cv::Mat img_l_gray;
    cv::Mat img_l_copy;

    // Transform RGB TO GRAY
    cv::cvtColor(frame, img_l_gray, cv::COLOR_BGR2GRAY);

    // Aruco detection
    cv::aruco::detectMarkers(img_l_gray, dictionary, corners, ids);

    if (ids.size()>0)
    {
        img_l_copy = img_l_gray.clone();
        cv::aruco::drawDetectedMarkers(img_l_copy, corners, ids);
    }
    return corners;

}

Eigen::MatrixXd better_points(std::vector<std::vector<cv::Point2f>> corner)
{
    Eigen::MatrixXd vector1(2,1);
    Eigen::MatrixXd vector4(2,1);
    Eigen::MatrixXd aux(2,1);

    Eigen::MatrixXd new_corner(2,4);
    double d, T;
    double angle;


    double corner_1x, corner_1y, corner_4x, corner_4y;

    vector1(0,0) = corner[0][0].x;
    vector1(1,0) = corner[0][0].y;

    vector4(0,0) = corner[0][3].x;
    vector4(1,0) = corner[0][3].y;

    aux = vector4 - vector1;
    angle = std::atan2(aux(0,0), aux(1,0));

    d = aux.norm();
    T = d/3;

    corner_1x = corner[0][0].x + T*std::sin(angle);
    corner_1y = corner[0][0].y + T*std::cos(angle);

    corner_4x = corner_1x + T*std::sin(angle);
    corner_4y = corner_1y + T*std::cos(angle);

    // new corners
    new_corner(0,0) = corner_1x;
    new_corner(1,0) = corner_1y;

    new_corner(0,1) = corner[0][1].x;
    new_corner(1,1) = corner[0][1].y;

    new_corner(0,2) = corner[0][2].x;
    new_corner(1,2) = corner[0][2].y;

    new_corner(0,3) = corner_4x;
    new_corner(1,3) = corner_4y;

    return new_corner;
}
Eigen::MatrixXd point_3d(std::vector<std::vector<cv::Point2f>> pixel_l, std::vector<std::vector<cv::Point2f>> pixel_r, cv::Mat mtxl, cv::Mat mtxr, double b){
     float fx_l, fy_l;
     Eigen::MatrixXd center;
     Eigen::MatrixXd aux_pixel_l;
     Eigen::MatrixXd aux_pixel_r;
     Eigen::MatrixXd center_l;
     Eigen::MatrixXd center_r;
     Eigen::MatrixXd disparity;
     Eigen::MatrixXd z(1,4);
     Eigen::MatrixXd x(1,4);
     Eigen::MatrixXd y(1,4);
     Eigen::MatrixXd Final(5,4);



     fx_l = mtxl.at<double>(0,0);
     fy_l = mtxl.at<double>(1,1);

     aux_pixel_l = better_points(pixel_l);
     aux_pixel_r = better_points(pixel_r);

     center_l = center_camera(aux_pixel_l, mtxl);
     center_r = center_camera(aux_pixel_r, mtxl);

     disparity = center_l - center_r;
     for(int i=0; i<disparity.cols(); i++){
        z(0,i) = 0.0774*(b*fx_l)/disparity(0,i);
        x(0,i) = (z(0,i)*center_l(0,i))/(fx_l*0.0774);
        y(0,i) = (z(0,i)*center_l(1,i))/(fx_l*0.0774);
     }

     Final(0,0) = aux_pixel_l(0,0);
     Final(0,1) = aux_pixel_l(0,1);
     Final(0,2) = aux_pixel_l(0,2);
     Final(0,3) = aux_pixel_l(0,3);


     Final(1,0) = aux_pixel_l(1,0);
     Final(1,1) = aux_pixel_l(1,1);
     Final(1,2) = aux_pixel_l(1,2);
     Final(1,3) = aux_pixel_l(1,3);


     Final(2,0) = x(0,0);
     Final(2,1) = x(0,1);
     Final(2,2) = x(0,2);
     Final(2,3) = x(0,3);
     
     Final(3,0) = y(0,0);
     Final(3,1) = y(0,1);
     Final(3,2) = y(0,2);
     Final(3,3) = y(0,3);

     Final(4,0) = z(0,0);
     Final(4,1) = z(0,1);
     Final(4,2) = z(0,2);
     Final(4,3) = z(0,3);
     
     return Final;
 }
Eigen::MatrixXd center_camera(Eigen::MatrixXd pixels, cv::Mat mtx)
{
    float uo, vo;
    Eigen::MatrixXd pixel_normado(3,4);
    Eigen::MatrixXd tranformation(3,3);
    Eigen::MatrixXd center(3,4);
    Eigen::MatrixXd pixel_final(2,4);

    uo = mtx.at<double>(0,2);
    vo = mtx.at<double>(1,2);

    // formation matrix pixel normado
    pixel_normado(0,0) = pixels(0,0);
    pixel_normado(1,0) = pixels(1,0);
    pixel_normado(2,0) = 1;

    pixel_normado(0,1) = pixels(0,1);
    pixel_normado(1,1) = pixels(1,1);
    pixel_normado(2,1) = 1;

    pixel_normado(0,2) = pixels(0,2);
    pixel_normado(1,2) = pixels(1,2);
    pixel_normado(2,2) = 1;

    pixel_normado(0,3) = pixels(0,3);
    pixel_normado(1,3) = pixels(1,3);
    pixel_normado(2,3) = 1;

    // Formation matrix tranformation
    tranformation(0,0) = 1;
    tranformation(0,1) = 0;
    tranformation(0,2) = -uo;

    tranformation(1,0) = 0;
    tranformation(1,1) = 1;
    tranformation(1,2) = -vo;
    
    tranformation(2,0) = 0;
    tranformation(2,1) = 0;
    tranformation(2,2) = 1;

    center = tranformation*pixel_normado;

    pixel_final(0,0) = center(0,0);
    pixel_final(0,1) = center(0,1);
    pixel_final(0,2) = center(0,2);
    pixel_final(0,3) = center(0,3);
    
    pixel_final(1,0) = center(1,0);
    pixel_final(1,1) = center(1,1);
    pixel_final(1,2) = center(1,2);
    pixel_final(1,3) = center(1,3);

    std::cout<< pixel_final <<std::endl;

    return pixel_final;
}