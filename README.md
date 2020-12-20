##CarND-Capstone
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

###Setting up:
You have the three options for setting up this project: Virtual Machine, Native Installation and docker. 
If you are running a windows machine like me, I would recommend using docker. Windows wsl2 (Windows Subsytem Linux) has tight integration with Docker Desktop which makes for a delightful development experience. 
I will be going over how I used docker to get started. If you want to use any of the other two options, please refer [here](https://github.com/udacity/CarND-Capstone/blob/master/README.md).

Steps:
1. Get [wsl2](https://docs.microsoft.com/en-us/windows/wsl/install-win10) on your windows machine.
2. Install [Docker Desktop](https://www.docker.com/products/docker-desktop).
3. Although not necessary, I would highly recommend getting [vscode](https://code.visualstudio.com/) and installing the [docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) and [remote development extension pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack). This would allow you to explore subystem directories, launch containers, mount directories and do a lot more right from your code editor.
4. Get the [code](https://github.com/nturumel/CarND-Capstone) and the [simulator](https://github.com/udacity/CarND-Capstone/releases).
5. Open wsl2 bash terminal.
6. Run: 
```bash 
    docker pull redherring2141/carnd-capstone
```
 to get the docker necessary image.

7. `cd` into the project directory. 
8.  Run  (might want to create a script for this):
```bash
    docker run -p 4567:4567 -v {pwd}:/capstone -v /tmp/log:/root/.ros/  --name={container_name} --rm -it    redherring2141/carnd-capstone:latest
```
9. If you want to set up another terminal, simply run:
```bash
    docker exec -it <container name>
 ```
 and run:
 ```bash
    source /root/ros_entrypoint.sh 
 ```
11.  `cd` into the ros directory of the container and run the following to launch the project:
```bash
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch
```
12.  Start the simulator, turn off **manual** mode and start the **camera**.     



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
