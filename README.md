### Getting started with Depth AI and ROS2

This project is an example of the implementation of an OAK-D camera and a Turtlebot. We will use ROS (Robot Operating System) and Depth AI packages/libraries to locate a person relative to the position of the camera and then instruct the turtlebot to follow that person. This guide will be done in more chapters and will explain (or instruct toward relevant guides) everything from setting up the turtlebot to the building of the ROS packages. 

I used VirtualBox VM image of Ubuntu 20.04, OAK-D S2 camera and turtlebot3 waffle-pi. ROS2 Galactic was the version of ROS used on the Ubuntu machine, while turtlebot was working with ROS2 Foxy. This guide should work for different set ups (with some modifications), but it was only tested on this configuration. 

# Setting up the Ubuntu machine and TurtleBot

There are 2 options for installing Galactic - via Debian package or from source. There is no major difference in which one you choose - just be aware of which file you need to source when you need to use ROS2. All sourcing will be provided assuming you are using bash, if you use shell you will have to source .sh files. We are using ROS2 Galactic is used because this project is meant to be integrated with Nav2 stack - and Galactic distribution of ROS is much better in dealing with nav2 than Foxy. 

[Debian Package Install](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

If you install ROS2 Galactic via Debian packages you will source it with this command:

```
  source /opt/ros/galactic/setup.bash 
```

[Building from source](https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Development-Setup.html)
Assuming you install ROS2 in the same package as the guide above you will source it from: 

```
. ~/ros2_galactic/install/local_setup.bash
```

Robotis page has a pretty [comprehensive guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) on how set up your PC and turtlebot so they can work together. Make sure you select Foxy option at the top and follow the guide.  Since we are using ROS2 Galactic on our Ubuntu machine "PC setup" that part of the guide will be slightly different for our installation. Since we already have ROS2 installed we can skip the first step and then install all dependent ROS2 packages with the following commands:

```
$ sudo apt-get install ros-galactic-gazebo-*
$ sudo apt install ros-galactic-cartographer
$ sudo apt install ros-galactic-cartographer-ros
$ sudo apt install ros-galactic-navigation2
$ sudo apt install ros-galactic-nav2-bringup
```

Then we can install turtlebot3 packages:

```
$ source ~/.bashrc
$ sudo apt install ros-galactic-dynamixel-sdk
$ sudo apt install ros-galactic-turtlebot3-msgs
$ sudo apt install ros-galactic-turtlebot3
```

Make sure to note somewhere what ROS_DOMAIN_ID you gave to the bot as that will be relevant in the future:

```
$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ source ~/.bashrc
```

[The second part and onwards](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/) of the guide linked before deals with setting up your turtlebot. TurtleBot uses Foxy distribution of ROS2, but distribution on TurtleBot is not very relevant and gives us an opportunity to show how to work between 2 different ROS versions. Make sure to give the bot the same ROS_DOMAIN_ID that you gave to the Ubuntu machine and make sure that the bot and the machine are on the same network. If you have trouble connecting your turtlebot to the internet [this thread](https://raspberrypi.stackexchange.com/questions/108636/setting-wifi-up-via-the-command-line-ubuntu-server-18-04-4-lts-raspberry-pi-4) can serve as a decent debugging tool - in my case my WIFI-SSID and WIFI_PASSWORD variables were not under quotes and I couldn't connect to my home network. Test if your VM and Raspberry PI on the turtlebot can ping each other. If you are using VirtualBox make sure to add 2nd adapter to the VM that is a Bridged Adapted in Promiscuous Mode.

![image1-VirtualBox](https://i.imgur.com/kd5obvO.png)

When you give cli command 
```
ip addr
```
you should be given a reponse that looks similar to this

![image2-VirtualBox](https://i.imgur.com/7LB0sUE.png)

As far as this guide is concerned only Bridged Adapter IP address is relevant. Bridged Adapter links your VM to the network your host PC is on, so if you are not experienced in networking just find the IP of your host PC and the IP address (out of the few returned by ip addr command) that is similar to it likely belongs to Bridged Adapter.

It could be useful to set ROS2 master and hostname for Raspberry Pi, so Raspberry Pi knows the IP of your Ubuntu machine. 

```
nano ~./bashrc
```

and then add the following lines at the end

```
export ROS_MASTER_URI=http://your ubnutu machine ip:11311
export ROS_HOSTNAME=your ubnutu machine ip
```



## Testing ROS2 connection

This project consists of 2 packages. One package is based on a simple publisher subscriber that is part of the [beginner guide](https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)  on ROS2 documentation. To get started with testing we need to make a ROS2 workspace. We need to make a folder and then clone this repo into that folder. Since the python code in this repo is already inside /src map (as required by ROS2) there are no other necessary steps. 

```
mkdir ~/ros2_follow_ws
git clone https://github.com/danilo-pejovic/follow-me-bot .
```

After that make sure you source your installation and then proceed with checking if you have all dependencies installed with rosdep and building the package (make sure you only build py_pubsub package since the other package has other dependencies that you might not have) 

```
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select py_pubsub
```

Now open a different terminal (it is very important that installation of a package and ROS2 installation are not sourced at the same time as it can lead to problems in some cases) in the ros workspace folder and source the build of our package and try tunning the publisher node:

```
. install/setup.bash
ros2 run py_pubsub talker
```

Follow the same procedure on Raspberry PI and run command below. Make sure both machines are on the same ROS domain.

```
ros2 run py_pubsub listener
```

You should see indications that machines can hear each other. If there are problems in communication common debugging method for communication between foxy and galactic is exporting the following environment variables inside the terminal you are running talker and listener node. 

```
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```



## Getting to know Depth AI

Now that we are done with ROS2 (for now) - let's move on onto Depth AI and OAK-D camera. I recommend installing [depth ai demo](https://github.com/luxonis/depthai) and playing around with it to get to know the environment. This will also install all requirements for controlling the turtlebot package. 

```
git clone https://github.com/luxonis/depthai.git
cd depthai
python3 install_requirements.py
python3 depthai_demo.py
```



 [Documentation](https://docs.luxonis.com/en/latest/#) is pretty extensive and shows many different use cases. If you are using Virtual Box you will likely run into the issue when starting the camera - as you will need to route USB to VM. [This guide](https://docs.luxonis.com/projects/api/en/latest/install/#virtual-box) should help you with setting up the camera if you are using VirtualBox. After you enable Intel loopback device in settings the same way you enabled Movidius device, demo will find OAK-D camera without any issues.  

Depth AI code generally consists of a [pipeline](https://docs.luxonis.com/projects/api/en/latest/components/pipeline/) in which you set up your camera and determine how you want it to act. This project takes advantage of Spatial Detection capabilities of OAK-D cameras. 

For this project pipeline is virtually the same as the pipeline for a [object tracker](https://github.com/luxonis/depthai-python/blob/main/examples/ObjectTracker/spatial_object_tracker.py) that is in a list of examples on Luxonis github page. 

## Getting everything to work together

Finally we are nearing the end of this guide. By now you should know the basics of how Depth AI works and how ROS2 works and have everything set up to run the other package in this project. Navigate back to the root folder of your ros2 workspace and build the whole project, don't forget to source ROS2 installation if needed:

```
cd ~/ros2_follow_bot
colcon build
```

 If the build is successful (you may get the warning that setup.py is deprecated but you can safely ignore that) you open another terminal and source installation of this package the same way you did with just publisher/subscriber package. Then run the code with

```
ros2 run tb3_vel controller
```

You should get a camera feed and the cli should print out person distance in millimeters along with the speed it is instructing the robot to go toward the person:

![image2-commandoutput](https://i.imgur.com/omD3eLd.png)

Now if we start the TurtleBot and do the bring up command:

```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Along with setting environment variables in Ubuntu terminal where we start our project:

```
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Command:

```
ros2 run tb3_vel controller
```

should successfully bring the robot toward a person it is following. I recommend testing the software first with a robot with its wheels taken off and then with ample free space so you can calibrate speed you need to limit your robot to.

This guide should soon be upgraded to include Nav2 navigation along with SLAM method to map out the space in which robot is located in. 
