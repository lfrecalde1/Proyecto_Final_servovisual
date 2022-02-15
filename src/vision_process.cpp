#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <cv_basics/image_data.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

static const std::string OPENCV_WINDOW = "Image window";

cv::FileStorage fsr("/home/fer/Webots_simulator/src/cv_basics/src/right_parameters.yml", cv::FileStorage::READ); 
cv::FileStorage fsl("/home/fer/Webots_simulator/src/cv_basics/src/left_parameters.yml", cv::FileStorage::READ); 

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


public:
  // Depth values topic
  ros::Publisher pixel_pub = nh_.advertise<cv_basics::image_data>("Mavic_2_PRO/Pixels_data", 10);
  cv_basics::image_data pixel_message;
  // Aruco Dictionay
  cv::Mat mtx_l, dist_l;
  cv::Mat mtx_r, dist_r;

  //Aruco Dictionary
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

  // Distance between cameras
  double B = 0.2;
  // Undistor matrices
  cv::Mat img_r;
  cv::Mat img_l;

  // name images
  std::string left = "left";
  std::string right = "right";

  // aux variable to split the image
  int h;
  int w;
  int half_w;


  ImageConverter(cv::FileStorage fsr)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/Mavic_2_PRO/camera_lr", 1,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter", 1);
    // Name window
    cv::namedWindow(OPENCV_WINDOW);

    // Read parameters camera
    fsr["K"] >> mtx_r;
    fsr["K"] >> mtx_l; 
    fsr["D"] >> dist_r;
    fsr["D"] >> dist_l;

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat current_frame = cv_ptr->image;
    cv::Mat frame_gray;

    // Aux image ariuco
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

    // Update GUI Window
    aux_l = aruco_process(img_l);
    aux_r = aruco_process(img_r);

    aux_l_corner = aruco_corners(img_l);
    aux_r_corner = aruco_corners(img_r);

    if (!aux_l.empty()){

        //finales_l = better_points(aux_l_corner, aux_l);
        system_coordinates = point_3d(aux_l_corner, aux_r_corner, mtx_l, mtx_r, B);
        //std::cout << "System coodenadas \n" << std::endl;
        //std::cout << system_coordinates << std::endl;
        cv::imshow(OPENCV_WINDOW, aux_l);
        cv::waitKey(3);
        send_message(system_coordinates);
    }
    else {
        send_message_vacio();

    }


    //send Data to the topic
    //send_message();
  }
  void send_message(Eigen::MatrixXd data)
  {
    int u1,v1,u2,v2,u3,v3,u4,v4;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4;
    int bandera = 1;
    u1 = (int)data(0,0);
    v1 = (int)data(1,0);
    x1 = data(2,0);
    y1 = data(3,0);
    z1 = data(4,0);

    u2 = (int)data(0,1);
    v2 = (int)data(1,1);
    x2 = data(2,1);
    y2 = data(3,1);
    z2 = data(4,1);

    u3 = (int)data(0,2);
    v3 = (int)data(1,2);
    x3 = data(2,2);
    y3 = data(3,2);
    z3 = data(4,2);


    u4 = (int)data(0,3);
    v4 = (int)data(1,3);
    x4 = data(2,3);
    y4 = data(3,3);
    z4 = data(4,3);

    pixel_message.u1 = u1;
    pixel_message.v1 = v1;
    pixel_message.x1 = x1;
    pixel_message.y1 = y1;
    pixel_message.z1 = z1;

    pixel_message.u2 = u2;
    pixel_message.v2 = v2;
    pixel_message.x2 = x2;
    pixel_message.y2 = y2;
    pixel_message.z2 = z2;

    pixel_message.u3 = u3;
    pixel_message.v3 = v3;
    pixel_message.x3 = x3;
    pixel_message.y3 = y3;
    pixel_message.z3 = z3;

    pixel_message.u4 = u4;
    pixel_message.v4 = v4;
    pixel_message.x4 = x4;
    pixel_message.y4 = y4;
    pixel_message.z4 = z4;

    pixel_message.flag = bandera;


    pixel_pub.publish(pixel_message);

  }
  
  void send_message_vacio()
  {
    int bandera = 0;
    pixel_message.flag = bandera;
    pixel_pub.publish(pixel_message);

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

    //std::cout<< pixel_final <<std::endl;

    return pixel_final;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Opencv_process");
  ImageConverter ic(fsr);
  ros::spin();
  return 0;
}
