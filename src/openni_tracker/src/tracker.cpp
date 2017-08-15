/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Johns Hopkins University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 */

// ROS Dependencies
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <iostream>
// NITE Dependencies
#include "NiTE.h"
#include "NiteSampleUtilities.h"
#include <XnContext.h>
#include <XnCppWrapper.h>
// Tracker Messages
#include <openni2_tracker/TrackerUser.h>
#include <openni2_tracker/TrackerUserArray.h>

// opencv Dependencies
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// Qt Dependencies
#include <QFile>
#include <QTextStream>

using namespace cv;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

QString dataline = "";
QString userdata = "";

#define USER_MESSAGE(msg) \
{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

typedef std::map<std::string, nite::SkeletonJoint> JointMap;

void ximage2opencv(const xn::ImageMetaData& xImageMap, cv::Mat& im)
{
    int h = xImageMap.YRes();
    int w = xImageMap.XRes();
    const XnRGB24Pixel* xx = xImageMap.RGB24Data();
    const cv::Mat tmp(h, w, CV_8UC3, (void*)xx);
    cv::cvtColor(tmp, tmp, CV_BGR2RGB);
    tmp.copyTo(im);
}

void updateUserState(const nite::UserData& user, unsigned long long ts){
    if (user.isNew())
        USER_MESSAGE("New")
                else if (user.isVisible() && !g_visibleUsers[user.getId()])
                USER_MESSAGE("Visible")
                else if (!user.isVisible() && g_visibleUsers[user.getId()])
                USER_MESSAGE("Out of Scene")
                else if (user.isLost())
                USER_MESSAGE("Lost")

                g_visibleUsers[user.getId()] = user.isVisible();


    if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
    {
        switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
        {
        case nite::SKELETON_NONE:
            USER_MESSAGE("Stopped tracking.")
                    break;
        case nite::SKELETON_CALIBRATING:
            USER_MESSAGE("Calibrating...")
                    break;
        case nite::SKELETON_TRACKED:
            USER_MESSAGE("Tracking!")
                    break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
            USER_MESSAGE("Calibration Failed... :-|")
                    break;
        }
    }
}


bool publishJointTF(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string j_name, nite::SkeletonJoint j, std::string tf_prefix, std::string relative_frame, int uid){
    if (j.getPositionConfidence() > 0.0){
        userdata.append(" ");
        userdata.append(QString::number(j.getPosition().x/1000.0));
        userdata.append(" ");
        userdata.append(QString::number(j.getPosition().y/1000.0));
        userdata.append(" ");
        userdata.append(QString::number(j.getPosition().z/1000.0));
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(j.getPosition().x/1000.0, j.getPosition().y/1000.0, j.getPosition().z/1000.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        std::stringstream frame_id_stream;
        std::string frame_id;
        frame_id_stream << "/" << tf_prefix << "/user_" << uid << "/" << j_name;
        frame_id = frame_id_stream.str();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relative_frame, frame_id));
    }
    return true;
}

int main(int argc, char** argv){

    system("touch /home/mohammad/catkin_ws/oniFiles/data_set.txt");
    QFile data_set("/home/mohammad/catkin_ws/oniFiles/data_set.txt");
    if(!data_set.open(QIODevice::ReadWrite | QIODevice::Text)){
        return 0;
    }
    QTextStream stream(&data_set);

    VideoWriter *rgbVideoWriter = new VideoWriter("/home/mohammad/catkin_ws/oniFiles/rgb.avi",CV_FOURCC('M','J','P','G'),60,Size(640,480),true);
    VideoWriter *depthVideoWriter = new VideoWriter("/home/mohammad/catkin_ws/oniFiles/depth.avi",CV_FOURCC('M','J','P','G'),60,Size(640,480),true);

    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); //private node handler
    std::string tf_prefix, relative_frame = "";
    tf::TransformBroadcaster br;

    // Get Tracker Parameters
    if(!pnh.getParam("tf_prefix", tf_prefix)){
        ROS_ERROR("tf_prefix not found on Param Server! Check your launch file!");
        return -1;
    }
    if(!pnh.getParam("relative_frame", relative_frame)){
        ROS_ERROR("relative_frame not found on Param Server! Check your launch file!");
        return -1;
    }
    openni::Device devDevice;
    openni::Status ret = openni::OpenNI::initialize();
    if ( ret != openni::STATUS_OK ) {
        std::cout << "##########\n";
    }
    ret = devDevice.open("/home/mohammad/catkin_ws/oniFiles/Captured2.oni");

    if ( ret != openni::STATUS_OK ) {
        std::cout << "!!!!!!!!!!!!!!!\n";
    }

    // NITE Stuff
    nite::Status niteRc;
    nite::NiTE::initialize();

    openni::VideoStream depthStream;
    ret = depthStream.create( devDevice, openni::SENSOR_DEPTH);

    openni::VideoStream vsColorStream;
    vsColorStream.create( devDevice, openni::SENSOR_COLOR);

    openni::VideoMode mMode;
    mMode.setResolution(800,800);
    mMode.setFps(30);
    mMode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );
    vsColorStream.setVideoMode(mMode);
    if( devDevice.isImageRegistrationModeSupported( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devDevice.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }

    vsColorStream.setMirroringEnabled(false);
    // Loop
    printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

    nite::UserTracker userTracker;
    nite::UserTrackerFrameRef userTrackerFrame;
    // while (!wasKeyboardHit()){
    niteRc = userTracker.create(&devDevice);

    if (niteRc != nite::STATUS_OK){
        printf("Couldn't create user tracker\n");
        return 3;
    }

    vsColorStream.start();
    depthStream.start();

    ros::Rate rate(30.0);
    int frame_id = 1;
    while (nh.ok()){
        niteRc = userTracker.readFrame(&userTrackerFrame);
        if (niteRc != nite::STATUS_OK){
            printf("Get next frame failed\n");
            continue;
        }
        // get color frame
        openni::VideoFrameRef vfColorFrame;
        cv::Mat mImageBGR;
        cv::Mat depthImage;
        cv::Mat mImageDepth;
        if(vsColorStream.readFrame( &vfColorFrame ) == openni::STATUS_OK )
        {
            // convert data to OpenCV format
            const cv::Mat mImageRGB( vfColorFrame.getHeight(), vfColorFrame.getWidth(), CV_8UC3, const_cast<void*>( vfColorFrame.getData() ) );
            // convert form RGB to BGR
            cv::cvtColor( mImageRGB, mImageBGR, CV_RGB2BGR );

            //            vfColorFrame.release();
        }

        openni::VideoFrameRef depthFrame;
        depthStream.readFrame( &depthFrame);
        if ( depthFrame.isValid()) {

            depthImage = cv::Mat( depthStream.getVideoMode().getResolutionY(),
                                  depthStream.getVideoMode().getResolutionX(),
                                  CV_16U, (char*)depthFrame.getData() );

            depthImage.convertTo( depthImage, CV_8U, 255.0 / 10000 );
            cv::cvtColor(depthImage,mImageDepth,CV_GRAY2BGR);
        }
        const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
        int trackd_users=0;
        for (int i = 0; i < users.getSize(); ++i){
            const nite::UserData& user = users[i];
            updateUserState(user,userTrackerFrame.getTimestamp());
            if (user.isNew()){
                userTracker.startSkeletonTracking(user.getId());
            } else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED){
                JointMap named_joints;
                named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD) );
                named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK) );
                named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER) );
                named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER) );
                named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW) );
                named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW) );
                named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND) );
                named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND) );
                named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO) );
                named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP) );
                named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP) );
                named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE) );
                named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE) );
                named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT) );
                named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT) );

                bool newTrackedUser = false;
                for (JointMap::iterator it=named_joints.begin(); it!=named_joints.end(); ++it){
                    nite::SkeletonJoint j = it->second;
                    if(j.getPositionConfidence() > 0.0){
                        if(newTrackedUser == false){
                            userdata.append("id").append(" ").append(QString::number(user.getId()));
                            newTrackedUser = true;
                        }
                        int x = j.getPosition().x;
                        int y = j.getPosition().y;
                        int z = j.getPosition().z;
                        float px , py;
                        userTracker.convertJointCoordinatesToDepth(x,y,z ,&py,&px);
                        userdata.append(" ");
                        userdata.append(QString(it->first.data())).append(" ").append(QString::number(int(px)));
                        userdata.append(" ").append(QString::number(int(py)));
                    }
                    publishJointTF(nh,br,it->first,it->second,tf_prefix,relative_frame,user.getId());
                }
                if(newTrackedUser == true){
                    trackd_users++;
                    userdata.append("\n");
                }
            }
        }
        if(trackd_users > 0){
            dataline.append(QString::number(frame_id).append(" ").append(QString::number(trackd_users))).append("\n");
            dataline.append(userdata);
            stream << dataline;
            dataline = "";
            userdata = "";
            trackd_users = 0;
        }
        cv::imshow("RGB Camera",mImageBGR);
        cv::imshow( "Depth Camera", mImageDepth);
        frame_id++;
        rgbVideoWriter->write(mImageBGR);
        depthVideoWriter->write(mImageDepth);
        cv::waitKey(1);
        rate.sleep();
    }
    nite::NiTE::shutdown();
}

/*

xn::Context context;
xn::Player xPlayer;
context.Init();
context.OpenFileRecording("/home/mohammad/catkin_ws/personRecognitionFiles/OpenNI2/Bin/x64-Release/Captured.oni",xPlayer);
xPlayer.SetRepeat(false);

xn::ImageGenerator xImageGenerator;
xImageGenerator.Create(context);
xImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);

xn::DepthGenerator depth_generator;
depth_generator.Create(context);

unsigned int numFrames;
xPlayer.GetNumFrames(xImageGenerator.GetName(), numFrames);
if (numFrames == 0)
{
    std::cerr << "No frames found!" << std::endl;
    return false;
}
context.StartGeneratingAll();

xn::ImageMetaData xImageMap;
xn::DepthMetaData xDepthMap;

xImageGenerator.GetMetaData(xImageMap);
depth_generator.GetMetaData(xDepthMap);
std::vector<cv::Mat> rgb;
cv::Mat imgRGB;

for (int i = 0 ; i < numFrames ; i++){
    xImageGenerator.WaitAndUpdateData();
    depth_generator.WaitAndUpdateData();
    ximage2opencv(xImageMap, imgRGB);

    rgb.push_back(imgRGB);
    cv::imshow("asd",imgRGB);
    cv::waitKey(1);
}

return 0;
while (nh.ok()){
    openni::Device device ;
    rc = device.open("file.oni");
    niteRc = userTracker.create(&device);

}
*/
//                    cv::circle(mImageBGR, Point(py,px),10, Scalar(0,0,255),CV_FILLED, 8,0);
