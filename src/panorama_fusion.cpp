/*
    SensorFusion PointCloud (Velodyne and Realsense)

    author : Yudai Sadakuni
*/

#include <ros/ros.h>

#include <fstream>
#include <string>
#include <typeinfo>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <bits/stdc++.h>
#include <math.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include <velodyne_camera_calibration/coloring.h>
using namespace std;
// ofstream ofs("test01.txt", ios_base::out);

template<typename T_p>
class SensorFusion{
    private:
        ros::NodeHandle nh;


        sensor_msgs::CameraInfo current_cinfo;
        sensor_msgs::PointCloud2 current_pc2;

        ros::Subscriber image_sub;
        ros::Subscriber cinfo_sub;
        ros::Subscriber lidar_sub;

        ros::Publisher pub_image;
        ros::Publisher pub_cloud;

        tf::TransformListener listener;
        tf::StampedTransform  transform;
        bool flag;

    public:
        SensorFusion();
        void image_callback(const sensor_msgs::Image::ConstPtr&);
        void cinfo_callback(const sensor_msgs::CameraInfo::ConstPtr&);
        void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr&);
        void sensor_fusion(const sensor_msgs::Image::ConstPtr);
        bool tflistener(std::string target_frame, std::string source_frame);

        cv::Point2d  project3dToPixel(cv::Point3d, cv::Mat);
};

template<typename T_p>
SensorFusion<T_p>::SensorFusion(): nh("~")
{

    // ofstream ofs("test01.txt", ios_base::out);
    // cout.rdbuf(ofs.rdbuf());


    image_sub = nh.subscribe("/image", 10, &SensorFusion::image_callback, this);
    cinfo_sub = nh.subscribe("/cinfo", 10, &SensorFusion::cinfo_callback, this);
    lidar_sub = nh.subscribe("/lidar", 10, &SensorFusion::lidar_callback, this);

    pub_image = nh.advertise<sensor_msgs::Image>("/projection", 10);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/colord_cloud", 10);
    flag = false;
}

template<typename T_p>
void SensorFusion<T_p>::image_callback(const sensor_msgs::Image::ConstPtr& image)
{
    // cout<<image->header.frame_id<<endl;
    // cout<<current_pc2.header.frame_id<<endl;
    if(!flag) tflistener(image->header.frame_id, current_pc2.header.frame_id);
    sensor_fusion(image);
}

template<typename T_p>
void SensorFusion<T_p>::cinfo_callback(const sensor_msgs::CameraInfo::ConstPtr& cinfo)
{
    current_cinfo = *cinfo;
}

template<typename T_p>
void SensorFusion<T_p>::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
    current_pc2 = *pc2;
}

template<typename T_p>
bool SensorFusion<T_p>::tflistener(std::string target_frame, std::string source_frame)
{
    ros::Time time = ros::Time(0);
    try{
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(4.0));
        listener.lookupTransform(target_frame, source_frame,  time, transform);
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(4.0).sleep();
        return false;
    }
}

template<typename T_p>
cv::Point2d SensorFusion<T_p>::project3dToPixel(cv::Point3d pt, cv::Mat rgb_image)
{
    cv::Point2d uv;
    float B;
    float L;
    float  width = rgb_image.cols;
    float height = rgb_image.rows;

    if(pt.x >= 0)
    {
        B = M_PI/2 - atan2(pt.x, pt.y);
    }
    else
    {
        B = -(M_PI/2 - atan2(pt.x, pt.y));
    }
    L = atan2(sqrt(pt.x*pt.x + pt.y*pt.y), pt.z);

    uv.x = width/2 + B*width/2*M_PI;
    uv.y = height/2 - L*height/M_PI;

    return uv;
}



template<typename T_p>
void SensorFusion<T_p>::sensor_fusion(const sensor_msgs::Image::ConstPtr image)
{
    //pcl::PointXYZI型　点群の位置と輝度情報
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(current_pc2, *velodyne_cloud);

    typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
    pcl::copyPointCloud(*velodyne_cloud, *cloud);

    // transform pointcloud from lidar_frame to camera_frame
    tf::Transform tf;
    tf.setOrigin(transform.getOrigin());
    tf.setRotation(transform.getRotation());
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);

    //cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(image)->image;

    // Realsense Data is saved BGR. change BGR to RGB
    cv::Mat rgb_image;
    // rgb_image = cv_image;
    cv::cvtColor(cv_image ,rgb_image, CV_BGR2RGB);

    // set PinholeCameraModel
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(current_cinfo);


    // SensorFusion Step
    typename pcl::PointCloud<T_p>::Ptr colored_cloud(new pcl::PointCloud<T_p>);
    *colored_cloud = *trans_cloud;
	cv::Mat projection_image = rgb_image.clone();

    for(typename pcl::PointCloud<T_p>::iterator pt=colored_cloud->points.begin(); pt<colored_cloud->points.end(); pt++)
    {

        cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
        cv::Point2d uv;
        uv = project3dToPixel(pt_cv, rgb_image);

        if(uv.x>0 && uv.x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows)
        {
            // Coloring PointCloud
            (*pt).b = rgb_image.at<cv::Vec3b>(uv)[0];
            (*pt).g = rgb_image.at<cv::Vec3b>(uv)[1];
            (*pt).r = rgb_image.at<cv::Vec3b>(uv)[2];
            // Projection PointCloud
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
            COLOUR c = GetColour(int(range/20*255.0), 0, 255);
            cv::circle(projection_image, uv, 1, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);

            //output_data
            int b=(*pt).b;
            int g=(*pt).g;
            int r=(*pt).r;

            // streambuf* last = cout.rdbuf();
            // cout.rdbuf(ofs.rdbuf());

            // cout<<(*pt).z<<"\t"<<b<<"\t"<<g<<"\t"<<r<<"\n";
            // cout.rdbuf(last);
        }
    }

    // for(typename pcl::PointCloud<T_p>::iterator pt=colored_cloud->points.begin(); pt<colored_cloud->points.end(); pt++)
    // {
    //
    //     if((*pt).z<0){
    //         (*pt).b = 255;
    //         (*pt).g = 255;
    //         (*pt).r = 255;
    //     }
    //     else{
    //         cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
    //         cv::Point2d uv;
    //         uv = cam_model.project3dToPixel(pt_cv);
    //
    //         if(uv.x>0 && uv.x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows)
    //         {
    //             // Coloring PointCloud
    //             (*pt).b = rgb_image.at<cv::Vec3b>(uv)[0];
    //             (*pt).g = rgb_image.at<cv::Vec3b>(uv)[1];
    //             (*pt).r = rgb_image.at<cv::Vec3b>(uv)[2];
    //             // Projection PointCloud
    //             double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
    //             COLOUR c = GetColour(int(range/20*255.0), 0, 255);
    //             cv::circle(projection_image, uv, 1, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
    //
    //             //output_data
    //             int b=(*pt).b;
    //             int g=(*pt).g;
    //             int r=(*pt).r;
    //
    //             streambuf* last = cout.rdbuf();
    //             cout.rdbuf(ofs.rdbuf());
    //
    //             cout<<(*pt).z<<"\t"<<b<<"\t"<<g<<"\t"<<r<<"\n";
    //             cout.rdbuf(last);
    //         }
    //         else{
    //             (*pt).b = 255;
    //             (*pt).g = 255;
    //             (*pt).r = 255;
    //         }
    //     }
    // }
    //
    // transform pointcloud from camera_frame to lidar_frame
    typename pcl::PointCloud<T_p>::Ptr output_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*colored_cloud, *output_cloud, tf.inverse());

    // Publish colored pointcloud
    sensor_msgs::PointCloud2 output_pc2;
    pcl::toROSMsg(*output_cloud, output_pc2);
    output_pc2.header.frame_id = current_pc2.header.frame_id;
    // output_pc2.header.frame_id = pc2->header.frame_id;
    output_pc2.header.stamp = ros::Time::now();
    pub_cloud.publish(output_pc2);

    // Publish Projection Image
    sensor_msgs::ImagePtr output_image;
    output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", projection_image).toImageMsg();
    output_image->header.frame_id = image->header.frame_id;
    output_image->header.stamp = ros::Time::now();
    pub_image.publish(output_image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_fusion");

    //pcl::PointXYZRGB型　点群の位置情報(xyz)と色の情報(RGB)
    SensorFusion<pcl::PointXYZRGB> cr;

    ros::spin();

    return 0;
}
