#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
using namespace ros;

const float range_resolution = 0.175;
Publisher radar_pub;

bool intensity_compare(pcl::PointXYZI a, pcl::PointXYZI b) 
{
    return a.intensity > b.intensity; 
}

pcl::PointCloud<pcl::PointXYZI>::Ptr create_radar_pc(Mat img)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc(new pcl::PointCloud<pcl::PointXYZI>);
    
    /*TODO : Transform Polar Image to Cartisien Pointcloud*/
    for(int col=0; col<img.cols; col++)
    {
        double azimuth_rad = static_cast<double>(col) / img.cols * 2.0 * M_PI;
        for(int row=0; row<img.rows; row++)
        {   
            pcl::PointXYZI point;

            if(row>5 && img.at<uchar>(row, col)>80)
            {
                point.x = static_cast<float>(row) * range_resolution * cos(azimuth_rad);
                point.y = static_cast<float>(row) * range_resolution * sin(azimuth_rad);
                point.z = 0;
                point.intensity = img.at<uchar>(row, col);
                new_pc -> push_back(point);
            }
        }
    }
    
    return new_pc;
}

void radarCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc_ptr = create_radar_pc(img);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*radar_pc_ptr, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "navtech";
    radar_pub.publish(pc_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_polar_to_pointcloud");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
    image_transport::Subscriber sub = it.subscribe("/Navtech/Polar", 1, radarCallback);
    
    ros::spin();
    return 0;
}