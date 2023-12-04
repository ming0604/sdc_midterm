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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>

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
    int image_rows = img.rows;
    int image_cols = img.cols;
    float radar_intensity;

    float delta_angle;
    float x, y, distance, angle;
    int k_strongest = 40;
    float intensity_threshold = 80;
    delta_angle = (2.0*M_PI)/image_cols;

       // go through all the radar image angles
    for (int i = 0; i < image_cols; i++)
    {   
        pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
        // get the angle
        angle = -delta_angle * i;

        // start from the 4th row
        for (int j = 4; j < image_rows; j++)
        {   
            pcl::PointXYZI point;
            // count the x and y values
            distance = range_resolution * j;
            x = distance * cos(angle);
            y = distance * sin(angle);

            //get the intensity of the pixel 
            radar_intensity = static_cast<float>(img.at<uchar>(j, i));

            // store data into a point (assume at z=0)
            point.x = x;
            point.y = y;
            point.z = 0.0;
            point.intensity = radar_intensity;

            if (point.intensity >= intensity_threshold && abs(point.x) < 50 && abs(point.y) < 50)
            {
                points->push_back(point);
            }
        }

        // sort the points by decreasing intensity
        std::sort(points->points.begin(), points->points.end(), intensity_compare);
        // k strongest filtering
        int points_to_keep = std::min(k_strongest, static_cast<int>(points->size()));
        for (int k = 0; k < points_to_keep; k++)
        {
            new_pc->push_back(points->points[k]);
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