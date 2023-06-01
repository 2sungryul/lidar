/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#define WIDTH   500
#define HEIGHT  500
#define RATIO ((WIDTH/2.0)/5000.0)  //5000mm->(WIDTH/2)pixel
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
using namespace sl;

void plotLidar(Mat& img, double dist, double angle);

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
	const char * opt_channel_param_first = "/dev/ttyUSBLidar";
	sl_u32         opt_channel_param_second = 115200;
    sl_result     op_result;
    struct timeval start,end1;
    double diff1;
    
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! \
			rtph264pay pt=96 ! udpsink host=203.234.58.121 port=8001 sync=true";
            
    cv::VideoWriter writer(dst, cv::CAP_GSTREAMER, 0, (double)10, cv::Size(WIDTH, HEIGHT), true);
	if(!writer.isOpened())  { cout << "Writer error" << endl; return -1;}
	
    // set signal handler 
    signal(SIGINT, ctrlc);
    
    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", "SL_LIDAR_SDK_VERSION");

	// create the driver instance
	ILidarDriver * drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // create the serial port channel
    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    IChannel* _channel;
    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_OK(op_result)) connectSuccess = true;        
    }
    if (!connectSuccess) {
        delete drv;
        drv = NULL;
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first);
        exit(-2);
    }
    
    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        delete drv;
        drv = NULL;
        fprintf(stderr, "Error, checkSLAMTECLIDARHealth\n");
        exit(-2);
    }
  
    // stop scan...
	drv->stop();    
    // start scan...
    drv->startScan(0,1);

    // generate image window
    Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
	//line(img, Point(WIDTH / 2, 0), Point(WIDTH / 2, HEIGHT - 1), Scalar(255, 0, 0));
	//line(img, Point(0, HEIGHT / 2), Point(WIDTH-1, HEIGHT / 2), Scalar(255, 0, 0));

    // fetech result and print it out...
    double dist;
    double angle;
    while (1) {
        gettimeofday(&start,NULL);
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = (int)(sizeof(nodes) / sizeof(nodes[0]));
        op_result = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(op_result)) {
            img = cv::Scalar(255,255,255); // clear window
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                dist = nodes[pos].dist_mm_q2/4.0f;
                angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                
                /*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    angle,
                    dist,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
*/
                //plot lidar data 
                plotLidar(img, dist, angle);	
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
        Mat gray;
        writer << img;
        //waitKey(1);
        gettimeofday(&end1,NULL);
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        printf("time1:%lf\n", diff1);
    }

    drv->stop();
	usleep(200*1000);
	drv->setMotorSpeed(0);
    printf("scan finished\n");
   
    return 0;
}

void plotLidar(Mat& img, double dist, double angle)
{
	Point2f cp(WIDTH / 2.0, HEIGHT / 2.0);
	Point2f pt(WIDTH / 2.0, HEIGHT / 2.0 + dist * RATIO);
	drawMarker(img, cp, Scalar(255, 0, 0), MARKER_CROSS, 10);
	
    //circle(img, pt, 3, Scalar(255,0,0),-1);
	Mat M = getRotationMatrix2D(cp, -angle, 1);
	double array[3] = { pt.x, pt.y, 1.0 };
	Mat ptM(3, 1, CV_64FC1, array);
	Mat rotpt = M * ptM;
	//cout << rotpt;
    //circle(img, cp, 2, Scalar(255, 0, 0), -1);
	circle(img, Point2f(rotpt.at<double>(0, 0), rotpt.at<double>(1, 0)), 1, Scalar(0, 0, 255), -1);
}
