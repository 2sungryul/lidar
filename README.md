# lidar

lidar test program on Jeson nano(rplidar a1)

Prerequisites : Opencv 4.6, Gstreamer, and rplidar a1 C++ SDK

# What to do on Jetson nano

install rplidar C++ SDK

$ cd ~

$ git clone https://github.com/Slamtec/rplidar_sdk.git 

$ cd rplidar_sdk 

$ make

download, build, and run on Jetson nano

$ cd ~

$ git clone https://github.com/2sungryul/lidar.git

$ cd lidar

$ make

$ sudo ./lidar

# What to do on Windows
For video streaming from robot camera, open the command prompt and download gst1.bat in a working directory

\> gst1.bat
