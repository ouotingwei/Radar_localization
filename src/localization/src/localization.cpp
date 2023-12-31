#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h> // add

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

#include"slidewindow.cpp"

using namespace std;

class Localizer
{
private:
    ros::NodeHandle _nh;

    ros::Subscriber radar_pc_sub;
    ros::Subscriber map_sub;
    ros::Subscriber gps_sub;

    ros::Publisher radar_pc_pub;
    ros::Publisher radar_pose_pub;
    ros::Publisher path_pub;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    nav_msgs::Path path;
    tf::TransformBroadcaster br;
    
    std::string save_path;
    std::ofstream file;

    float pose_x;
    float pose_y;
    float pose_yaw;
    float gps_x;
    float gps_y;
    float gps_yaw;

    int seq = 0;
    int max_iter;
    float epsilon1;
    float epsilon2;
    float correspond;

    Eigen::Matrix4f init_guess;

    bool map_ready = false;
    bool gps_ready = false;
    bool initialized = false;

    float last_x;
    float last_y;

public:
    Localizer(ros::NodeHandle nh) : map_pc(new pcl::PointCloud<pcl::PointXYZI>)
    {
        map_ready = false;
        gps_ready = false;
        
        _nh = nh;
        _nh.param<string>("/save_path", save_path, "/Default/path");

        init_guess.setIdentity();
        file.open(save_path);
        file << "id,x,y,yaw\n";

        radar_pc_sub = _nh.subscribe("/radar_pc", 1, &Localizer::radar_pc_callback, this);
        map_sub = _nh.subscribe("/map_pc", 1, &Localizer::map_callback, this);
        gps_sub = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);

        radar_pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/tranformed_radar_pc", 1);
        radar_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/tranformed_radar_pose", 1);
        path_pub = _nh.advertise<nav_msgs::Path>("/localization_path", 1);
    }

    ~Localizer()
    {
        ROS_WARN("Exit Localization");
        file.close();
    }

    void gps_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        //ROS_WARN("Got GPS data");

        initialized = false;    // add
        gps_ready = false;

        gps_x = msg->pose.position.x;
        gps_y = msg->pose.position.y;
        tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double r, p, yaw;
        m.getRPY(r, p, yaw);
        gps_yaw = yaw;

        if(!gps_ready)
        {
            pose_x = gps_x;
            pose_y = gps_y;
            pose_yaw = gps_yaw;
            gps_ready = true;
        }
    }

    void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        //ROS_WARN("Got Map Pointcloud");
        pcl::fromROSMsg(*msg, *map_pc);
        map_ready = true;
    }

    void radar_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {   
        //ROS_WARN("Got Radar Pointcloud");
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);   //add
        pcl::fromROSMsg(*msg, *radar_pc);
        //ROS_INFO("point size: %d", radar_pc->width);

        float search_range = 4;
        float best_x, best_y, best_yaw = 0;
        float min_scores = 1000;

        while(!(map_ready && gps_ready))
        { 
            //ROS_WARN("Wait for map and gps ready");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if (!initialized)
        {   
            for(float x = pose_x - search_range; x <= pose_x + search_range; x += 4){
                for(float y = pose_y - search_range; y <= pose_y + search_range; y += 4){
                    pcl::copyPointCloud(*radar_pc, *pc);
                    set_init_guess(x, y, pose_yaw);
                    pcl::transformPointCloud(*pc, *pc, init_guess);

                    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_init;
                    icp_init.setInputSource(pc);
                    icp_init.setInputTarget(map_pc);

                    //icp_init.setMaximumIterations(500);
                    icp_init.setMaxCorrespondenceDistance(2);
                    //icp_init.setEuclideanFitnessEpsilon(0.1);

                    icp_init.align(*output_pc);

                    if( icp_init.getFitnessScore() < min_scores && icp_init.hasConverged()){
                        min_scores = icp_init.getFitnessScore();
                        best_x = x;
                        best_y = y;
                    }
                }
            }

            set_init_guess(best_x, best_y, pose_yaw);
            pose_x = best_x;
            pose_y = best_y;
            initialized = true;
            
            set_init_guess(pose_x, pose_y, pose_yaw);
            initialized = true;
        }

        //pcl::transformPointCloud(*radar_pc, *radar_pc, init_guess);
        
        // icp Rough Matching
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_R;
        icp_R.setInputSource(radar_pc);
        icp_R.setInputTarget(map_pc);

        icp_R.setMaximumIterations(100);
        icp_R.setMaxCorrespondenceDistance(4);
        icp_R.setEuclideanFitnessEpsilon(1e-4);
        icp_R.setTransformationEpsilon(1e-4);

        icp_R.align(*output_pc, init_guess);

        if (icp_R.hasConverged()) {
            std::cout << "ICP Rough Matching converged. Transformation matrix:\n" << icp_R.getFinalTransformation() << std::endl;
            ROS_WARN("ICP Rough Matching converged. Fitness score: %f", icp_R.getFitnessScore());

            pose_x = icp_R.getFinalTransformation()(0, 3);
            pose_y = icp_R.getFinalTransformation()(1, 3);
        
            // Extract yaw (rotation around the z-axis) using Euler angles
            pose_yaw = atan2(icp_R.getFinalTransformation()(1, 0), icp_R.getFinalTransformation()(0, 0));

            set_init_guess(pose_x, pose_y, pose_yaw);

        } else {
            std::cout << "ICP Rough Matching did not converge." << std::endl;
            return;
        }

        tf_brocaster(pose_x, pose_y, pose_yaw);
        radar_pose_publisher(pose_x, pose_y, pose_yaw);

        sensor_msgs::PointCloud2 radar_pc_msg;
        pcl::toROSMsg(*radar_pc, radar_pc_msg);
        radar_pc_msg.header.stamp = ros::Time::now();
        radar_pc_msg.header.frame_id = "base_link";
        radar_pc_pub.publish(radar_pc_msg);
        ROS_INFO("Publish transformed pc");
        ROS_INFO("[seq %d] x:%.3f, y:%.3f, yaw:%.3f\n", seq, pose_x, pose_y, pose_yaw);

        file << seq << ",";
        file << pose_x << ",";
        file << pose_y << ",";
        file << pose_yaw << "\n";

        seq++;
    }

    void radar_pose_publisher(float x, float y, float yaw)
    {
        geometry_msgs::PoseStamped pose;
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, yaw);
        myQuaternion.normalize();

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = myQuaternion.getX();
        pose.pose.orientation.y = myQuaternion.getY();
        pose.pose.orientation.z = myQuaternion.getZ();
        pose.pose.orientation.w = myQuaternion.getW();
        radar_pose_pub.publish(pose);
        
        path.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    void tf_brocaster(float x, float y, float yaw)
    {  
        ROS_INFO("Update map to baselink");
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, 0) );
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    }

    void set_init_guess(float x, float y, float yaw)
    {
        init_guess(0, 0) = cos(yaw);
        init_guess(0, 1) = -sin(yaw);
        init_guess(0, 3) = x;

        init_guess(1, 0) = sin(yaw);
        init_guess(1, 1) = cos(yaw);
        init_guess(1, 3) = y;

        //ROS_WARN("set init_guess ");
    }
};


int main(int argc, char** argv) 
{
    ros::init (argc, argv, "localizer");
    ros::NodeHandle nh;
    Localizer Localizer(nh);

    ros::spin();
    return 0;
}