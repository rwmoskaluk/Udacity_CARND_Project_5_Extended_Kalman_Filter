# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

NOTES for Windows 10 Setup
---
Initially following the Udacity project help for setting everything up to run on a Windows computer did not result in uWebSocket working.  What I ended up doing in order to get this project to work is as follows:

1) Follow the instructions for installing Linux Bash (I installed Ubuntu version on my Windows 10 Machine)
https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/
2) After creating a UNIX username and password run `sudo apt-get update`
3) Once updated I ran `sudo apt-get install git`
4) `sudo apt-get install cmake`
5) `sudo apt-get install openssl`
6) `sudo apt-get install libssl-dev`
7) `git clone https://github.com/rwmoskaluk/Udacity_CARND_Project_5_Extended_Kalman_Filter`
8) `cd Udacity_CARND_Project_5_Extended_Kalman_Filter`
9) `chmod a+x install-linux.sh`
10) `./install-linux.sh`
11) This ran for a bit but did not appear to install uWebSocket, to install uWebSocket I manually did the following
12) `git clone https://github.com/uWebSockets/uWebSockets `
13) `cd uWebSockets `
14) `git checkout e94b6e1`
15) `mkdir build`
16) `cd build`
17) `cmake ..`
18) `make`
19) `sudo make install` (uWebSockets should now be installed under /usr/lib64)
20) switch back over to the Udacity project folder
21) create build folder in the Udacity project folder if it doesn't exist already
22) `cd build`
23) `cmake ..`
24) `make`
25) `./ExtendedKF` to run the project, should see listening port 4567

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

# Video Output
###For Dataset 1
<img src="video_output/dataset_1.gif?raw=true" width="720px">

###For Dataset 2
<img src="video_output/dataset_2.gif?raw=true" width="720px">


---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

