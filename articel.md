Recently I have been tinkering with idea of using Raspberry Pi Pico as a micro-controller working with Raspberry Pi 4 for robotics. I have seen these two articles that are very good in documenting the step-by-step process so kudos to those guys.

Canonical Ubuntu Blog Article
Official micro-ROS repo readme page
However, after following through steps on both of these sources — the Ubuntu blog article and the official github repo page, I still ran into some show stopper compilation errors and I could not find real resolution for that on the web. So I decided to create this quick article to help anyone out there who is in same situation as me to get the basic example working.

My approach ended up being a combination of the approaches described in those two sources in following ways to get the Pico Micro-ROS working on Ubuntu 20.04 machine:

Ubuntu article suggests to use a single root directory to house both the pico-sdk and the example source code. And uses the VS Code UI to compile the code. The directory structure was fine but the VS Code UI compiling really did not work for me in spite of trying to do various combinations of path in environment variable in settings.json file within the VS Code. May be it has to do with me not understanding the context and the settings that need to be set properly for VS Code. On the other hand, the approach suggested by official github repo to compile using command line worked well so I went with that for compilation part.
The official Github repo suggests to use a docker image to run the micro-ROS agent. I didn’t really like that idea so I went with approach suggested by Ubuntu blog article to use snap plugin for micro-ROS agent and run the micro-ROS agent as Ubuntu snap.
Essentially, I combined the steps from both of these articles to get it working for me [without using VS Code to compile and without docker] and these are the steps I followed. Hope this works for you.

At high level these are the steps and all these are done on laptop/computer:

Install C++ build tools and set pre-requisites like env variables
Download the pico sdk code and micro-ROS example code from github repos
Build the code that will generate a uf2 file that can be copied to Pico
Plug-in Pico to show up as USB drive and copy the uf2 file generated for the example code
Install and configure snap plugin for micro-ROS agent
Connect to Pico using snap over serial port
Start the micro-ROS example application on Pico
Confirm the ROS2 node/messages are showing up on laptop from Pico
Prep for Pico
BTW, this is regular Pico without W or wifi support. Before you start on steps make sure you have your Pico with headers soldered mounted on a breadboard and USB cable ready to connect between your laptop and Pico. Just keep in mind, when you plug the laptop end of the USB cable to hold down the white BOOTSEL button as you are plugging in the USB cable into laptop [assuming the other micro-USB end is already plugged in to Pico side]. This will make the Pico show up as USB drive which we will need to copy the .uf2 file from our build output folder to Pico after the compilation is successful.

Steps to follow on laptop or your main computer
Assumption — it is running Ubuntu 20.04 and ROS2 foxy is installed

Step 1 — install pre-requisites

sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi doxygen git python3
setup env variables or alternatively you can open ~/.bashrc file using editor of your choice and add two export lines at the end of the file, save and source the ~/.bashrc file.

# Configure environment variables
echo "export PICO_TOOLCHAIN_PATH=..." >> ~/.bashrc
echo "export PICO_SDK_PATH=$HOME/micro_ros_ws/src/pico-sdk" >> ~/.bashrc
source ~/.bashrc
Step 2 — Download the pico sdk code and micro-ROS example code from github repos

mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
### clone the pico-sdk
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
### clone the micro-ros example repo. This example simply publishes an incrementing integer value every second
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
The git clone steps will take a while, so be patient, its > 400mb.

Your directory structure should look like this:

~
--micro_ros_ws
----src
------pico-sdk
------micro_ros_raspberrypi_pico_sdk
Step 3— Build the code that will generate a uf2 file that can be copied to Pico

cd micro_ros_raspberrypi_pico_sdk
mkdir build
cd build
cmake ..
make
Step 4— Plug-in Pico to show up as USB drive and copy the uf2 file generated for the example code

Get RoboFoundry’s stories in your inbox
Join Medium for free to get updates from this writer.

Enter your email
Subscribe
At this point hold the BOOTSEL button down on Pico and plugin the USB cable into your computer. It should show up as USB drive.

Cope the .uf2 file generated from build step earlier to Pico like this

cp build/pico_micro_ros_example.uf2 /media/$USER/RPI-RP2
Step 5— Install and configure snap plugin for micro-ROS agent

Snap should already be installed if you are running Ubuntu 20.04, if not install it.

install micro-ros agent snap, enable hotplug feature and restart snap service

sudo snap install micro-ros-agent
sudo snap set core experimental.hotplug=true
sudo systemctl restart snapd
Step 6— Connect to Pico using snap over serial port

Make sure Pico is plugged in and run following to confirm serial is working properly

$ snap interface serial-port
name:    serial-port
summary: allows accessing a specific serial port
plugs:
  - micro-ros-agent
slots:
  - snapd:pico (allows accessing a specific serial port)
connect the plug and slot for serial

snap connect micro-ros-agent:serial-port snapd:pico
Step 7— Start the micro-ROS example application on Pico

sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200
If the LED light does not light up, unplug and plug the USB cable back. Wait for few seconds and it should show that the micro-ROS node is starting to publish

Note: I got it to work by running with sudo for now but will have to investigate a way to add correct privileges for my running user so it can be executed without sudo

[1660569056.713593] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1660569057.217852] info     | Root.cpp           | create_client            | create                 | client_key: 0x28B733E9, session_id: 0x81
[1660569057.218006] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x28B733E9, address: 0
[1660569057.272859] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x28B733E9, participant_id: 0x000(1)
[1660569057.277002] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x28B733E9, topic_id: 0x000(2), participant_id: 0x000(1)
[1660569057.279417] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x28B733E9, publisher_id: 0x000(3), participant_id: 0x000(1)
[1660569057.281894] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x28B733E9, datawriter_id: 0x000(5), publisher_id: 0x000(3)
Step 8— Confirm the ROS2 node/messages are showing up on laptop from Pico

At this point we should be able to see the messages published by ROS2 publisher on Pico from the main computer/laptop.

ros2 topic list
### will show something like this - our topic is /pico_publisher
/parameter_events
/pico_publisher
/rosout
Listen to the topic and echo the messages, it basically publishes an incrementing number at 1 second interval.

ros2 topic echo /pico_publisher
### should show something like this
data: 22
---
data: 23
---
data: 24
---
data: 25
---
data: 26
---
data: 27
See it in action
