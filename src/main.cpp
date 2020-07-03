#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//最低424*240 6fps 最高640*480 30fps
#define WIDEH 424
#define HEIGHT 240
#define FPS 6


void measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile)
{
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义图像中心点
    cv::Point center(color.cols/2,color.rows/2);
    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x)){
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }
    cout<<"遍历完成，有效像素点:"<<effective_pixel<<endl;
    float effective_distance=distance_sum/effective_pixel;
    cout<<"目标距离："<<effective_distance<<" m"<<endl;
    char distance_str[30];
    sprintf(distance_str,"the distance is:%f m",effective_distance);
    cv::rectangle(color,RectRange,Scalar(0,0,255),2,8);
    cv::putText(color,(string)distance_str,cv::Point(color.cols*0.02,color.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),2,8);
}

int main(int argc, char** argv) try
{
    // judge whether devices is exist or not
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0)
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();

    //
    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, WIDEH, HEIGHT, RS2_FORMAT_BGR8, FPS);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDEH, HEIGHT, RS2_FORMAT_Z16, FPS);

    // start stream
    rs2::pipeline_profile profile = pipe.start(cfg);//指示管道使用所请求的配置启动流

    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    auto intrinDepth = depth_stream.get_intrinsics();
    auto intrinColor = color_stream.get_intrinsics();

    auto extrinDepth2Color = depth_stream.get_extrinsics_to(color_stream);

    while(1)
    {
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架

        // Align to depth
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        frames = align_to_depth.process(frames);


        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();



        // Creating OpenCV Matrix from a color image
        Mat color(Size(WIDEH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat pic_depth(Size(WIDEH,HEIGHT), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        // Display in a GUI
//        namedWindow("Display Image", WINDOW_AUTOSIZE );
//        imshow("Display Image", color);
//        waitKey(1);
//        imshow("Display depth", pic_depth*15);
//        waitKey(1);

        int iLowH = 35;
        int iHighH = 120;
        int iLowS = 90;
        int iHighS = 255;
        int iLowV = 90;
        int iHighV = 255;//设置蓝色的颜色参量。
        Mat imgHSV;
        vector<Mat> hsvSplit;

        cvtColor(color, imgHSV, COLOR_BGR2HSV);
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2],hsvSplit[2]);
        merge(hsvSplit,imgHSV);

        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
        GaussianBlur(imgThresholded,imgThresholded, Size(3, 3), 0, 0);

//        imshow("滤波后的图像", imgThresholded);

        Mat tempImage=color.clone();
        vector<vector<Point>>contours;
        findContours(imgThresholded,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        vector<Rect>rect(contours.size());
        int x,y;
        for(int i=0;i<contours.size();i++)
        {
            rect[i]=boundingRect(contours[i]);
            x=rect[i].x;
            y=rect[i].y;
            int width=rect[i].width;
            int height=rect[i].height;
            rectangle(tempImage,Point(x,y),Point(x+width,y+height),Scalar(0,255,0),2);
        }
        rs2::depth_frame depth_ = frames.get_depth_frame();
        float dis_to_object = depth_.get_distance(x,y);
        cout<<"Object\n"<<"( "<<x<<","<<y<<","<< dis_to_object <<" )"<<endl;


        imshow("result",tempImage);
        waitKey(1);
        imshow("Display depth", pic_depth*15);
        waitKey(1);

        measure_distance(color,pic_depth,Size(20,20),profile);
    }
    return 0;
}

// error
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


