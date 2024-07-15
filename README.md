# ENPM663 - BUILDING A MANUFACTURING ROBOTIC SOFTWARE SYSTEM 

## RWA4 - Group 3
### Team

|Name|UID|Github ID| Email ID
|---|:---:|:---:|:---:|
|Shantanu Parab|119347539|[Link](https://github.com/shantanuparabumd)|sparab@umd.edu|



https://github.com/user-attachments/assets/c2290e07-981d-468a-bca0-afcd02e80931





## Dependencies Installation
Run the below commands to install required packages
```sh
sudo apt install python3-pykdl
pip install ultralytics
```
## Normal Testing

#### Sensors used 
- Advance Logical Camera

#### Instructions to run the package
Download the package and place it in `ariac_ws/src`
```sh
cd ~/ariac_ws
rm -rf buid/ log/ install/
colcon build
```
After successful build, source the package
```sh
source install/setup.bash
```
Start the ariac environment with advance logical cameras
```sh
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa4_group3 sensor_config:=sensors trial_name:=rwa4_spring2024
```
After all the parts and trays are loaded run the below command (Ideally wait for 5 seconds to spawn all parts and trays)

```sh
ros2 launch rwa4_group3 rwa4.launch.py
```

## Bonus Testing

#### Sensors used 
- RGB Camera
- Basic Logical Camera 

#### Instructions to run the package

Start the ariac environment
```sh
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa4_group3 sensor_config:=sensors_bonus trial_name:=rwa4_spring2024
```
After all the parts and trays are loaded run the below command (Ideally wait for 5 seconds to spawn all parts and trays)

```sh
ros2 launch rwa4_group3 rwa4_bonus.launch.py
```


Test
