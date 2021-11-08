#pragma once
#include <iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/dnn.hpp>
#include<fstream>
#include<string>
#include<time.h>
#include<thread>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<image_transport/image_transport.h>
#include<ros/package.h>
#include<opencv2/stitching/detail/exposure_compensate.hpp>


#define YOLO 0
using namespace std;

class Fisheye
{
public:
    Fisheye();
};

//enum class has to be here rather than in Fisheye.cpp
enum class TYPE
{
    L, M, R
};

struct BoundingBox
{
    // 18:33
    cv::Rect roi;
    int classID;
    double confidence;
    int boxID;
    int trackID;
};

struct YoloStruct
{
    vector<cv::Mat>netOutput;
    vector<int> classIds;
    vector<float> confidences;
    vector<int>indices;
    vector<BoundingBox>bBoxes;

    const float nmsThreshold = 0.2f;
    float confThreshold = 0.40f;
    vector<cv::Rect> boxes;
    double scalefactor;
    cv::Size size;
    
    cv::Mat blob;
    cv::Scalar mean;
    bool swapRB ;
    bool crop;
    cv::dnn::Net net;
    vector<cv::String>names;
    vector<string> classes;
};

void create_corresponding_landmarks(const int height, const int width, const int step,const TYPE T,
    const vector<cv::Mat>&warpingTemplates, vector<cv::Point>& landmarks_points,
    vector<cv::Point>& landmarks_points2, vector<vector<int>>& indexes_triangles);

int extract_index_vector(const vector<cv::Point2f>& vec, const cv::Point2f& pt);
void get_landmarks_after_warping(const TYPE T, const vector<cv::Mat>& warpingTemplates,vector<cv::Point>&landmark_points);
void warp_image(const cv::Mat& img,cv::Mat&dst, const vector<cv::Point>& landmarks_points,
    const vector<cv::Point>& landmarks_points2, const vector<vector<int>>& indexes_triangles);
void get_colorList(vector<cv::Scalar>&colorList);
void get_each_row_coordinates(const cv::Mat& img,const cv::Scalar&color, vector<cv::Point>&);
bool sort_coordinates(const cv::Point& a, const cv::Point& b);
cv::Mat generate_mask(const string &path);
vector<cv::Mat> get_warping_templates(const string& path1, const string& path2,const string& path3);
vector<cv::Mat> get_mask_templates(const string& path1, const string& path2, const string& path3);
void get_destination_shape(const string& path, int& h, int& w);
void panorama(const vector<cv::Mat>& maskTemplates,
    const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& img3, cv::Mat& output);

void warp_video(const vector<cv::Mat>& warpingTemplates, const vector<cv::Mat>& maskTemplates,const int step);

void yolo_detection(const cv::Mat& inputImg, const cv::Mat&frame_pano, vector<cv::Mat>& netOutput, vector<int>& classIds,
    vector<float>& confidences, vector<int>& indices, vector<BoundingBox>& bBoxes,
    const double scalefactor,const cv::Size& size, vector<cv::Rect>& boxes,  cv::Mat& blob,
    const cv::Scalar& mean,const bool swapRB,const bool crop, cv::dnn::Net& net, const vector<cv::String>& names,
    const vector<string>& classes, const float confThreshold, const float nmsThreshold);

class Listener
{
    public:
        cv::Mat m_frame1;
        cv::Mat m_frame2;
        cv::Mat m_frame3;
        cv::Mat temp;
        double resize_factor=0.5;
        void callback_L(const sensor_msgs::CompressedImageConstPtr&msg);
        void callback_M(const sensor_msgs::CompressedImageConstPtr&msg);
        void callback_R(const sensor_msgs::CompressedImageConstPtr&msg);

};
// void chatterCallback(dynamic_tutorials::TutorialsConfig &config);

void exposure_compensate(cv::Mat& src1, cv::Mat& src2, cv::Mat& src3,
    vector<cv::Point>& corners, vector<cv::UMat>& masks
    , cv::Ptr<cv::detail::ExposureCompensator>& compensator);
void equalizing_hist(cv::Mat&inputOutput);

