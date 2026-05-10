# WATonomous ASD Admission Assignment

Demo Video : https://youtu.be/9vyOPqbYeCc

# 1. Quick Start

## Prerequisites
* ROS 2 (Humble/Foxy)
* C++17
* Wato Developer Environment (watod)
* OrbStack (or Docker)

## Build and Run

clone the workspace and run the following code:
```bash
# Build the entire workspace
./watod build

# Launch the docker env
./watod up
```
then enable foxglove app and select `wato_asd_training_foxglove_config ` config file

To publish the transformation from `map` frame to `sim_world` frame, enter your docker environment and run the following code:

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id sim_world --child-frame-id map
```


# 2. Parameters

**Control Node :**
* `lookahead_distance` = 0.8
* `linear_speed` = 0.4
* `L` = 1.3

**Costmap Node :**
* `inflation_rad` = 1.3

**MapMemory Node :**
* `distance_threshold` = 0.2
* `moving_threshold` > 30


