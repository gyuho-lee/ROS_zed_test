//ROS Header
#include <ros/ros.h>

//ROS image transprt Header
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OpenCV Header
#include <opencv2/opencv.hpp>

//To calculate frame per second.
#include <time.h>

class ZedPublisher{
public:
    ZedPublisher();
    ~ZedPublisher();

    bool Setup(cv::VideoCapture cap, int width, int height, int fps);
    cv::Mat Cam_info(const cv::FileNode node , std::string filename , std::string resolution);
    cv::Mat inMat1_, distMat1_, inMat2_ , distMat2_;
    cv::Mat R, T;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat map1x_ , map1y_, map2x_ , map2y_;
    cv::Mat StereoCal(cv::Mat inMat1_ , cv::Mat inMat2_ ,cv::Mat R, cv::Mat T, int widht, int height);

    cv::Mat HomoMatrix;
    cv::Mat Wrap;
    cv::Mat Panorama;

private:
    ros::NodeHandle nh_;

    image_transport::Publisher raw_img_pub_;
    image_transport::Publisher left_img_pub_;
    image_transport::Publisher right_img_pub_;
    image_transport::Publisher pano_img_pub_;

    cv::FileNode node;
    std::string info_file;
    std::string resolution;

    double BaseLine, CV_HD, RX_HD, RZ_HD;


    std::string img_type;

    int dev;    //device number;
    int width;  //image width;
    int height; //image height;
    int fps;    //image frame per seconds

    cv::VideoCapture cap;

    cv::Mat raw_img;
    cv::Mat left_img;
    cv::Mat right_img;

    cv::Mat remap1_, remap2_;
  
    sensor_msgs::ImagePtr raw_img_msg_;
    sensor_msgs::ImagePtr left_img_msg_;
    sensor_msgs::ImagePtr right_img_msg_;
    sensor_msgs::ImagePtr pano_img_msg_;

};


//Main loop
ZedPublisher::ZedPublisher():nh_("~")
{
    ROS_INFO("Sensors package starting");
    image_transport::ImageTransport it_(nh_);  
   
    raw_img_pub_ = it_.advertise("/zed/raw_img",1);
    left_img_pub_ = it_.advertise("/zed/left_img",1);
    right_img_pub_ = it_.advertise("/zed/right_img",1);
    pano_img_pub_ = it_.advertise("/zed/panorama",1);


    nh_.param<int>("device",dev,0);
    ROS_INFO("device             : /dev/video%d",dev);
    nh_.param<int>("image_width",width,1280);
    ROS_INFO("image width        : %d" , width);
    nh_.param<int>("image_height",height,720);
    ROS_INFO("image height       : %d" , height);
    nh_.param<int>("image_fps",fps,60);
    ROS_INFO("image fps          : %d", fps);
    nh_.param<std::string>("img_type",img_type , "bgr8");
    ROS_INFO("image type         : %s" , img_type.c_str() );
   
    nh_.param<std::string>("cam_info",info_file , "SN18875.yaml");
    ROS_INFO("file name          : %s" , info_file.c_str() );
    nh_.param<std::string>("resolution",resolution , "HD"); //2K || FUD || HD || VGA
    ROS_INFO("camera resolution  : %s" , resolution.c_str() );
   
    //cv::namedWindow("raw_image",1);

    cv::VideoCapture cap(dev);
    Setup(cap,width,height,fps);
    Cam_info(node,info_file , resolution);
    StereoCal(inMat1_,inMat2_,R,T,width,height);
    double h_[] = { 0.967776746268218, 0.01687696542004494, 81.9565644825179
                    ,-0.007086890685699608, 0.9710208781657136 , 8.316012528397037
                    , -2.654999809134362e-05, -1.750287163834191e-05, 1};

    HomoMatrix = cv::Mat(3,3,CV_64FC1 , h_);
    try
    { 

        double st=0, end = 0 , fps =0;

        while(nh_.ok())
        {

            cap >> raw_img;
           
            left_img = raw_img(cv::Rect(0,0,width,height));
            right_img = raw_img(cv::Rect(width,0,width,height));

            cv::remap(left_img, remap1_, map1x_ ,map1y_ , cv::INTER_LINEAR , CV_HAL_BORDER_CONSTANT , cv::Scalar());
            cv::remap(right_img, remap2_, map2x_ ,map2y_ , cv::INTER_LINEAR , CV_HAL_BORDER_CONSTANT , cv::Scalar());

            cv::warpPerspective(remap2_, Wrap, HomoMatrix , cv::Size(remap2_.cols + remap1_.cols,remap2_.rows) , CV_INTER_CUBIC);
            Panorama = Wrap.clone();

            cv::Mat ROI(Panorama , cv::Rect(0,0,remap1_.cols , remap1_.rows));
            remap1_.copyTo(ROI);

            //cv::imshow("Panorama" , Panorama);

            //raw_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), img_type, raw_img).toImageMsg();
            left_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), img_type, remap1_).toImageMsg();
            right_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), img_type, remap2_).toImageMsg();
            pano_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), img_type, Panorama).toImageMsg();

            //raw_img_pub_.publish(raw_img_msg_);
            left_img_pub_.publish(left_img_msg_);
            right_img_pub_.publish(right_img_msg_);
            pano_img_pub_.publish(pano_img_msg_);


            cv::waitKey(10);

                       
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

ZedPublisher::~ZedPublisher()
{
    cap.release();
    cv::destroyAllWindows();
}


/*
@@brief : Camera setup(image size / frame rate)
@@        Return bool type
*/
bool ZedPublisher::Setup(cv::VideoCapture cap , int width, int height, int fps)
{
   
    if(!cap.isOpened())
    {
        ROS_INFO(" Camera is not opened");
        return false;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH , width*2);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT , height);
    cap.set(CV_CAP_PROP_FPS , fps);
   
    return true;
}

/*
@@brief : To read camera information
@@        distortion / focal length / principal point / R|T matrix of stereo
*/
cv::Mat ZedPublisher::Cam_info(const cv::FileNode node , std::string filename , std::string resolution)
{
    cv::FileStorage fs;
    fs.open(filename , cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        ROS_INFO(" [%s] Failed to open " , filename.c_str() );
    }

    cv::FileNode fn;
    cv::FileNodeIterator it = fn.begin(), it_end = fn.end(); // Go through the node
    for (; it != it_end; ++it)
        std::cout << (std::string)*it << std::endl;

    fs["LEFT_CAM_HD"] >> inMat1_;
    fs["RIGHT_CAM_HD"] >> inMat2_;

    fs["LEFT_CAM_HD_D"] >> distMat1_;
    fs["RIGHT_CAM_HD_D"] >> distMat2_;

    double BaseLine,CV_HD, RX_HD, RZ_HD;
    fs["BaseLine"] >> BaseLine;
    fs["CV_HD"] >> CV_HD;
    fs["RX_HD"] >> RX_HD;
    fs["RZ_HD"] >> RZ_HD;

    //std::cout << "BaseLine : " << BaseLine << std::endl;
    //std::cout << "CV_HD    : " << CV_HD << std::endl;
    //std::cout << "RX_HD    : " << RX_HD << std::endl;
    //std::cout << "RZ_HD    : " << RZ_HD << std::endl;

    std::vector<double> rvec;
    rvec.push_back(RZ_HD);
    rvec.push_back(CV_HD);
    rvec.push_back(RX_HD);
    cv::Rodrigues(rvec,R,cv::noArray());

    double tvec[] = {-BaseLine, 0,0};
    T = cv::Mat(3,1,CV_64FC1,tvec);

    //std::cout << "Rotation    matrix\n" << R << std::endl;
    //std::cout << "Translation matrix\n" << T << std::endl;

    return inMat1_,distMat1_ ,  inMat2_ , distMat2_ , R, T;
}

cv::Mat ZedPublisher::StereoCal(cv::Mat inMat1_ , cv::Mat inMat2_ ,cv::Mat R, cv::Mat T , int widht, int height)
{
    //std::cout << " inMat1_ " << std::endl << inMat1_ << std::endl;
    //std::cout << "distMat1_" << std::endl << distMat1_ << std::endl;

    //std::cout << " inMat2_ " << std::endl << inMat2_ << std::endl;
    //std::cout << "distMat2_" << std::endl << distMat2_ << std::endl;

    //std::cout << "    R    " << std::endl << R << std::endl;
    //std::cout << "    T    " << std::endl << T << std::endl;

    cv::stereoRectify(inMat1_,distMat1_, inMat2_,distMat2_,cv::Size(width,height),R,T,R1,R2,P1,P2,Q);
    ROS_INFO("StereoRectify calibrtaion ");
   
    //std::cout << "  R1 " << std::endl << R << std::endl;
    //std::cout << "  R2 " << std::endl << R << std::endl;
    //std::cout << "  P1 " << std::endl << R << std::endl;
    //std::cout << "  P2 " << std::endl << R << std::endl;
    //std::cout << "  Q  " << std::endl << R << std::endl;

    cv::initUndistortRectifyMap(inMat1_,distMat1_,R1,P1,cv::Size(width,height),CV_32FC1,map1x_,map1y_);
    cv::initUndistortRectifyMap(inMat2_,distMat2_,R2,P2,cv::Size(width,height),CV_32FC1,map2x_,map2y_);

    return map1x_, map1y_, map2x_ , map2y_;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"zed_opencv");

    ZedPublisher zed;   

    return 0;
} 