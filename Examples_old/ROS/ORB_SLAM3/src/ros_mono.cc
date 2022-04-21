/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/QuaternionStamped.h>
#include<sensor_msgs/Temperature.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
private:
    ros::NodeHandle nodeHandlerPub;
    ros::Publisher pose_pub;
    ros::Publisher status_pub;
    ros::Publisher init_orien_pub;

    geometry_msgs::PoseStamped slam_pose;
    sensor_msgs::Temperature slam_state;
    geometry_msgs::QuaternionStamped initBluerovOrientation;


    Sophus::SE3f Tcw;
    Sophus::SE3f Twc;
    std::chrono::steady_clock::time_point t0;
    std::chrono::steady_clock::time_point t1;
    double time_counter = 0.f;
    double fps = 0.f;
    int status;

public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void InitPub() {
        pose_pub = nodeHandlerPub.advertise<geometry_msgs::PoseStamped>("/ORB_SLAM3/Pose", 5);
        status_pub = nodeHandlerPub.advertise<sensor_msgs::Temperature>("/ORB_SLAM3/State", 5);
        init_orien_pub = nodeHandlerPub.advertise<geometry_msgs::QuaternionStamped>("/ORB_SLAM3/InitOrientation", 5);

        t0 = std::chrono::steady_clock::now();
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabBluerovPose(const sensor_msgs::ImuConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);
    igb.InitPub();
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    ros::Subscriber bluerov_pose_sub = nodeHandler.subscribe("/imu", 1, &ImageGrabber::GrabBluerovPose, &igb);


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabBluerovPose(const sensor_msgs::ImuConstPtr& msg) 
{
    initBluerovOrientation.header.frame_id = "world";
    initBluerovOrientation.header.stamp = msg->header.stamp;
    initBluerovOrientation.quaternion = msg->orientation;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    int new_status = mpSLAM->getTrackingState();

    if (new_status == 2 && status != 2) { // tracking established
        // publish the initial orientation of bluerov 
        init_orien_pub.publish(this->initBluerovOrientation);
    }
    
    status = new_status;

    Twc = Tcw.inverse();
    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    slam_state.header.frame_id = "slam";
    slam_state.header.stamp = cv_ptr->header.stamp;
    slam_state.temperature = status; // we are cutting some corners and using the temperature message to contain the status message. Might fix later. 
    status_pub.publish(this->slam_state);


    slam_pose.header.frame_id = "world";
    slam_pose.header.stamp = cv_ptr->header.stamp;
    // slam_pose.header.seq = 1; //status;

    slam_pose.pose.position.x = twc(0);
    slam_pose.pose.position.y = twc(1);
    slam_pose.pose.position.z = twc(2);
    slam_pose.pose.orientation.x = q.x();
    slam_pose.pose.orientation.y = q.y();
    slam_pose.pose.orientation.z = q.z();
    slam_pose.pose.orientation.w = q.w();
    pose_pub.publish(this->slam_pose);
    

}


