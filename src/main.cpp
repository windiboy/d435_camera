#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//最低424*240 6fps 最高640*480 30fps
#define width 424
#define height 240
#define fps 6


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
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    // start stream
    pipe.start(cfg);//指示管道使用所请求的配置启动流


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
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat pic_depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        // Display in a GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", color);
        waitKey(1);
        imshow("Display depth", pic_depth*15);
        waitKey(1);

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

        imshow("滤波后的图像", imgThresholded);

        Mat tempImage=color.clone();
        vector<vector<Point>>contours;
        findContours(dstImage,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        vector<Rect>rect(contours.size());
        for(int i=0;i<contours.size();i++)
        {
            rect[i]=boundingRect(contours[i]);
            int x=rect[i].x;
            int y=rect[i].y;
            int width=rect[i].width;
            int height=rect[i].height;
            rectangle(tempImage,Point(x,y),Point(x+width,y+height),Scalar(0,255,0),2);
        }
        imshow("result",tempImage);

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


