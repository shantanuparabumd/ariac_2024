## Complete package

### RWA3 -> rwa3_group3

### RWA4 -> rwa4_group3



## Bonus Testing (RGB Camera  + Basic Logical with YOLO)

Run ariac as follows to load rgb and logical cameras.

```sh
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa4_group3 sensor_config:=sensors_bonus trial_name:=rwa4_spring2024
```

Note that the sensors_bonus yaml has the sensor settings

Run the sensor_manager_bonus node first

```sh
ros2 launch rwa4_group3 rwa4_bonus.launch.py
```

This will launch all neccessary nodes. Just start the competition and let all the parts and trays spawn before starting this.


## Normal Testing (Advance Logical Camera )

Code for normal execution using Advance Logical Camera also exist in the package.


Start the ariac environment with advance logical cameras

```sh
ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=rwa4_group3 sensor_config:=sensors trial_name:=rwa4_spring2024
```

After all the parts and trays are loaded run the below command

```sh
ros2 launch rwa4_group3 rwa4.launch.py
```

