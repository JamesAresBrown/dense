//
// Created by hzx on 2022/9/12.
//

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pdense.h"

using namespace std;

queue<nav_msgs::Odometry> odometry_buf;
queue<sensor_msgs::ImagePtr> image_buf;
std::mutex m_buf;

Pdense pdense("../../../src/VINS-Fusion/dense/config/dense1.yaml", "1");

void global_image_callback(const sensor_msgs::ImagePtr& global_image) {
    m_buf.lock();
    image_buf.push(global_image);
    m_buf.unlock();
}

void global_odometry_callback(const nav_msgs::Odometry& global_odomatry) {
    m_buf.lock();
    odometry_buf.push(global_odomatry);
    m_buf.unlock();
}

[[noreturn]] void sync_process() {
    int num = 0;
    while(true)
    {
        m_buf.lock();
        if (!odometry_buf.empty() && !image_buf.empty()) {
            double odometry_time = odometry_buf.front().header.stamp.toSec();
            double image_time = image_buf.front()->header.stamp.toSec();
            if (odometry_time < image_time - 0.003) {
                odometry_buf.pop();
                cerr << "Throw odometry." << endl;
            }
            else if (odometry_time > image_time + 0.003) {
                image_buf.pop();
                cerr << "Throw image." << endl;
            }
            else {
                Eigen::Quaterniond q;
                q.x() = odometry_buf.front().pose.pose.orientation.x;
                q.y() = odometry_buf.front().pose.pose.orientation.y;
                q.z() = odometry_buf.front().pose.pose.orientation.z;
                q.w() = odometry_buf.front().pose.pose.orientation.w;
                Eigen::Matrix3d R = q.normalized().toRotationMatrix();

                Eigen::Vector3d T;
                T.x() = odometry_buf.front().pose.pose.position.x;
                T.y() = odometry_buf.front().pose.pose.position.y;
                T.z() = odometry_buf.front().pose.pose.position.z;

                sensor_msgs::ImageConstPtr image_msg = image_buf.front();
                cout << image_msg->encoding << endl;
                cv_bridge::CvImageConstPtr ptr;
//                if (image_msg->encoding == "8UC1") {
//                    sensor_msgs::Image img;
//                    img.header = image_msg->header;
//                    img.height = image_msg->height;
//                    img.width = image_msg->width;
//                    img.is_bigendian = image_msg->is_bigendian;
//                    img.step = image_msg->step;
//                    img.data = image_msg->data;
//                    img.encoding = "8UC2";
//                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC2);
//                }
//                else
                    ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC3);

                cv::Mat img = ptr->image.clone();

                cout << "odometry : " << odometry_time << "  " << "image: " << image_time << endl<< R << endl << T << endl;
                pdense.write(R, T, img);
                odometry_buf.pop();
                image_buf.pop();
                ++num;
            }
        }
        m_buf.unlock();


        if (num >= pdense.max_num_) {
            pdense.generateOption();
            pdense.pmvs2();
            pdense.PLY2PCD();
            pdense.Global();
            break;
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dense");
    ros::NodeHandle n("~");
    cout << "dense" << endl;
//    Pdense pdense("../../../src/VINS-Fusion/dense/config/dense1.yaml", "1");

    ros::Subscriber sub_global_odometry = n.subscribe("/globalEstimator/global_odometry", 100, global_odometry_callback);
    ros::Subscriber sub_global_image = n.subscribe("/gps_image", 1000, global_image_callback);

    std::thread sync_thread{sync_process};
    ros::spin();
    return 0;
}

