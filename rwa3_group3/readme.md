# ENPM663 - ARIAC - Group 3
Project repository of Group 3 for the ENPM663 course and ARIAC competition.

## Authors

|Name|Email|
|:---:|:---:|
|Akashkumar Parmar|akasparm@umd.edu|
|Bharat Bora Reddy|bbora@umd.edu|
|Neha Marne|nmarne@umd.edu|
|Sai Teja|saitejag@umd.edu|
|Shantanu Parab|sparab@umd.edu|
## Installing ARIAC (Prerequisite)
Run the following code snippet for quick installation, alternatively, refer to the [ARIAC documentation](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/getting_started/installation.html) incase of errors.

Installing Dependencies for ARIAC
```sudo apt install python3-rosdep
sudo apt install openjdk-17-jdk
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
sudo apt install python3-colcon-common-extensions
```
ARIAC Package installation
```
source /opt/ros/galactic/setup.bash
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src
git clone https://github.com/usnistgov/ARIAC.git src/ariac
cd ~/ariac_ws
colcon build --packages-select ariac
source install/setup.bash
```

## rwa3_group3 package installation
- Clone
   - Either clone the Repository in the colcon workspace of your choice using the command
   ```
   cd ~/<your-ws>/src
   git clone git clone https://github.com/shantanuparabumd/rwa3_group3
   ```
   OR
   - Create a new workspace and clone the repository in the src folder using following command
   
   ``` 
    mkdir –p ~/colcon_ws/src
    cd ~/colcon_ws/src
    git clone git clone https://github.com/shantanuparabumd/rwa3_group3
    ```
- Build and source
```
    cd ~/colcon_ws/
    colcon build 
    source insta/setup.bash    
```
## Running the competition
- Launch the ARIAC environment in one terminal.

```
source ~/ariac_ws/install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024
```

### Starting the CCS package
Start the CCS to perform tasks in a second terminal.
```
source ~/ariac_ws/install/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 run rwa3_group3 ccs_manager

```



## Troubleshooting
If you encounter issues, make sure all dependencies are installed and the ROS environment is properly sourced.
Ensure the rwa3_spring2024.yaml file is placed in the correct trials folder within the ariac_gazebo package.
Double-check service and topic names as they need to match those defined within the ARIAC competition environment.

