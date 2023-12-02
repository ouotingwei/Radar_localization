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
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc(new pcl::PointCloud<pcl::PointXYZI>);


    
    /*TODO : Transform Polar Image to Cartesian Pointcloud*/
    for(int col=0; col<img.cols; col++)
    {
        double azimuth_rad = static_cast<double>(col) / img.cols * 2.0 * M_PI;

        // Create a vector to store points for each angle
        std::vector<pcl::PointXYZI> points_for_angle;

        for(int row=0; row<img.rows; row++)
        {   
            pcl::PointXYZI point;

            if(row > 4 && img.at<uchar>(row, col) > 70)
            {
                // Calculate distance from the origin (circle center)
                double distance = static_cast<double>(row) * range_resolution;

                // Set a threshold for the distance from the origin
                double distance_threshold = 50.0;  // Adjust this value based on your requirements

                // Check if the distance exceeds the threshold
                if (distance <= distance_threshold)
                {
                    point.x = static_cast<float>(row) * range_resolution * cos(azimuth_rad);
                    point.y = static_cast<float>(row) * range_resolution * (-sin(azimuth_rad)); // Flip horizontally
                    point.z = 0;
                    point.intensity = img.at<uchar>(row, col);

                    // Add the point to the vector
                    points_for_angle.push_back(point);
                }
            }
        }

        // Sort the vector based on intensity in descending order
        std::sort(points_for_angle.begin(), points_for_angle.end(), intensity_compare);

        // Add the top five points with maximum intensity to the new point cloud
        int num_points_to_keep = std::min(40, static_cast<int>(points_for_angle.size()));
        for (int i = 0; i < num_points_to_keep; i++) {
            new_pc->push_back(points_for_angle[i]);
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