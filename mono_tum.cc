/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

//#include "~/ORB_SLAM2/include/System.h"
#include "System.h"
#include <ctello.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <Converter.h>
#include <pthread.h>
#include <unistd.h>

#define DEGREE 20

const int FRAME_NUMBER{5};
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?overrun_nofatal=1&fifo_size=500000"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

using namespace std;
using namespace ORB_SLAM2;

// this idea of using this pramter is from Dnieal, Dnieal and Yahav group 
bool finish=0; //if finishing the scaning


void saveMap(ORB_SLAM2::System &SLAM);

void* buildingMap(void* m);


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_point_data" << endl;
        return 1;
    }

    pthread_t thread;//for building map
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    

    Tello tello{};
    if (!tello.Bind()) return 0;

    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse())) ;
    //VideoCapture capture{TELLO_STREAM_URL, cv::CAP_ANY};
    pthread_create(&thread, NULL, buildingMap, argv);
    sleep(7);
	
    tello.SendCommand("takeoff"); // take off the quadcopter
    while(!tello.ReceiveResponse());
    tello.SendCommand("up 25");
    while(!tello.ReceiveResponse());
    sleep(10);
    for(int i=0; i<18; i++)
    {
        tello.SendCommand("cw 20");//rotate quadcopter 24 degrees
	while(!tello.ReceiveResponse());
	sleep(10);
	//tello.SendCommand("forward 25");//go forward 30 cm
	//while(!tello.ReceiveResponse());
	//tello.SendCommand("back 25");//back to center 
	//while(!tello.ReceiveResponse());
    }
    tello.SendCommand("cw 15");//rotate quadcopter 15 degrees
    while(!tello.ReceiveResponse());
    sleep(7);
    
    finish=1;
    pthread_join(thread, NULL);
    
    
    tello.SendCommand("land");//landing
    while(!tello.ReceiveResponse());
    return 0;

}

void* buildingMap(void* m)
{
    char** argv = (char**)m;
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    VideoCapture capture{TELLO_STREAM_URL, cv::CAP_ANY};

    // Main loop
    cv::Mat im;
    int i = 0;
    while(!finish){
        capture >> im;

        if(im.empty())
        {
            cerr << endl << "Failed to load image" << endl;
        }
        // this condition is from Dnieal, Dnieal and Yahav group
        else if(i%10 == 0)
        {
            SLAM.TrackMonocular(im,0);
        }
        i++;
    }

    // Stop all threads
    SLAM.Shutdown();
    saveMap(SLAM);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void saveMap(ORB_SLAM2::System &SLAM){
    cout << "saving map" << endl << endl;
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}

