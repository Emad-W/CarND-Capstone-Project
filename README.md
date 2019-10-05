# Self-Driving Car - Udacity CarND Capstone - System Integration Project
**This is the final project for the Udacity Self-Driving Car Engineer Nanodegree  It's done on an INDIVIDUAL basis  and NOT part of a TEAM.**
 


[//]: # (Image References)
[image1]: ./imgs/system_architecture.png
[image2]: ./imgs/nodes-design.png
[image3]: ./imgs/template-matching.jpeg
[image4]: ./imgs/Sample1.png
[image5]: ./imgs/sample2.png


## Project Overview

### Carla Architecture
Udacity has converted a car into a self-driving car which them named carla.  It's self-driving system is broken down into four major sub-systems: **Sensors**, **Perception**, **Planning** and **Control** 

![][image1]


#### Sensors
Includes everything needed to understand its surroundings and location including **cameras**, **lidar**, **GPS**, **radar**, and **IMU**
#### Perception
Abstracts sensor inputs into object **detection** and **localization**
##### Detection
* Includes software pipelines for vehicle detection, traffic light detection, obstacle detection, etc
* Techniques in image manipulation include Image matching, image search, feature extraction, color transforms.
* Methods of classification include color mappingalong with bounding boxes.
##### Localization
* Detecting the car location with respect to the surroundings and the environment.
* Onboard sensors are used to estimate transformation between measurements and a given map
#### Planning
Path planning is broken down into for sub-components: **route planning**, **prediction**, **behavioral planning**, and **trajectory planning**
##### Route Planning
The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.
##### Prediction
The prediction component estimates what actions other objects might take in the future according to run time events like vehicles were detected infront of us and what the prediction component would be for its future trajectory.
##### Behavioral Planning
The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
##### Trajectory Planning
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
### Control
The control component takes trajectory outputs and processes them with a controller algorithm like **PID** or **MPC** to adjust the control inputs for smooth operation of the vehicle. 


### ROS Architecture

The ROS Architecture consists of many nodes (written in Python and C++) that communicate with each other via messages. The nodes and the high level architecture are depicted in the picture below. The direction of the arrows clarifies the respective flow of communication. 

![][image2]

These systems consists of many nodes that all together communicate with the Car simulator (or Carla) to share the ros messages (topics) providing information about the car's state and surroundings (car's current position, velocity and images of the front camera) and receiving control input (steering, braking, throttle). These nodes can be associated with the three central tasks Perception, Planning and Control. 

The images get processed within the traffic light classifier by a trained neural network in order to detect traffic lights. The percepted state of a potentially upcoming traffic light is passed to the traffic light detector as well as the car's current pose and a set of base waypoints coming from the waypoint loader. With this frequently incoming information the traffic light detector is able to publish a waypoint close to the next traffic light where the car should stop in case the light is red. 

With the subscribed information of the traffic light detector and the the subscriptions to base waypoints, the waypoint updater node is able to plan acceleration / deceleration and publish it to the waypoint follower node. This node publishes to the DBW (Drive by wire) node that satisfies the task of steering the car autonomously. It also takes as input the car's current velocity (coming directly from the car / simulator) and outputs steering, braking and throttle commands. 



### Node Design

In this paragraph it will be talked about the node design of those nodes that are built within this project. Those are the waypoint updater(waypoint_updater.py), the traffic light detector (tl_detector.py) and traffic light classifier (tl_classifier.py).
The rest of the nodes are Twist controller and Drive by wire node which consistes of yaw controller, low pass filter and a PID to control the steering, brakes and velocity and send the commands according to the control signals coming from the waypoint updater node.

#### Waypoint Updater
The waypoint updater node is the main node as it determines the waypoints the car should follow. This node is responsible on updating the waypoints the velocity of the car which the velocity and brakes are determined by. This node serves as the hub that collects all the needed information in the perception and sensing phases and prepare them control signals to the controller accordingly.


#### Traffic Light Detection and classification
The TL detection node serves as the detection point of the comming traffic light signals, it determines whether there is a near TL or not and accordingly starts sending the data to the clasisfier to know the TL state whether it's RED, YELLOW or GREEN.
Later on the TL detection node sends message to the Waypoint updater according to the classified state of the TL that includes how far the TL is and the state for the Waypoint updater to handle.

Our Detector is implemented in the means of image search and feature matching that looks for a template of the traffic light inside the image.
After finding the TL sign we then crop it and sample the lights accordingly in the HSV domain to get the green and red components and compare them together to get the TL state.


![][image3]



## Simulation Results

After testing the developed algorithm and optimizing it's parameters, it reached a good classification accuracy for TL but due to the limitation in the performance and the intensive processing required from both the VM and the simulator, I reduced the sampling of the images of the TL so it can somehow achive realtime performance with decent prediction accuracy.

![][image4] Above is a sample of the car running test track running.

![][image5] Another sample while the car has stoped correctly detecting the Red Traffic light.







# Usability and system requirements
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
