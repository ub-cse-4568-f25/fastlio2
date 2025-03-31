<div align="center">
    <h1>SPARK-FAST-LIO2</h1>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/ROS2-Jazzy-blue" /></a>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <br />
    <br />
  <br />
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/763bafef-c11a-4412-a9f7-f138fc12ff9f" alt="KISS Matcher" width="95%"/></p>
  <p><strong><em>LiDAR mapping = SPARK-FAST-LIO2 + <a href="https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros">KISS-Matcher-SAM</a></em></strong></p>
</div>

______________________________________________________________________

## :package: Installation

## How to build

Put the code in your workspace/src folder

```shell
cd ${YOUR_COLCON_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/spark-fast-lio2.git
colcon build --packages-up-to spark_fast_lio
```

## How to run

We provide two out-of-the-box ROS2 examples

### 🏟️ Construct your colosseum

```
ros2 launch spark_fast_lio mapping_vbr_colosseo.launch.yaml
```

### 🇺🇸 Construct MIT campus

```
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=acl_jackal
```

### How to run `spark-fast-lio2` using your own ROS2 bag?

1. Copy `config/velodyne_mit.yaml` or `config/ouster_vbr.yaml` to `config/${YOUR_CONFIG}.yaml`, and set the appropriate values for:
   - `lidar_type`, `scan_line`, `timestamp_unit`, and `filter_size_map` depending on your sensor type
   - `extrinsic_T` and `extrinsic_R` (it's LiDAR w.r.t. IMU, i.e., `extrinsic * cloud w.r.t. LiDAR ->  cloud w.r.t. IMU`)
1. Configure your launch file and remap the lidar and imu topic names to match your setup.
   - Also set an appropriate rviz setup
1. Run your launch file, for example: `ros2 launch spark_fast_lio ${YOUR_LAUNCH}.launch.yaml`

### How to run with [KISS-Matcher-SAM](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros)?

TBU

______________________________________________________________________

## What's new? Key features and updates

TBU

For **DCIST** project,

```
roslaunch spark_fast_lio mapping_hamilton.launch robot_name:="hamilton"
```

Q. Why do we need to specify the robot name?
Because in `10_14` datatset, those are recorded by `acl_jackal2` (hidden secret by Yun).
For this reason, just in case, I separated and parameterized the sequence_name and robot_name.

## Note for MIT Guys (especially for SPARK)

### Tip 1

Originally, the estimated poses of FAST-LIO2 is **the IMU frame**.
However, for our pipeline, the `+x`, `+y`, and `+z` directions of the pose need to be forward, left, and up, respectively.
Therefore, I set the visualization_frame parameter to adjust the final coordinates accordingly (see [here](https://github.mit.edu/SPARK/spark_fast_lio/blob/6cdcb73f88003dbc27db9e365bc49f6c5d6e3f10/ada_lio/config/dcist/hamilton.yaml#L11)).

### Tip 2

To follow the tip 1 above, `lidar_frame` and `base_frame` should be provided by `/tf_static`.

So before you run the launch file, please check whether `tf` exists by using the following command:

```
rosrun tf tf_echo ${BASE_FRAME} ${LiDAR_FRAME}
// e.g.,
rosrun tf tf_echo apis/base apis/ouster_link
```

### Tip 3

To substitute LOCUS with SPARK-FAST-LIO, you can easily do it by using the following command:

```
    <!-- Lidar Odometry (LOCUS) -->
<!--    <include file="$(find dcist_spark_core)/launch/perception/locus/locus.launch" if="$(arg run_lidar_odom)">-->
<!--        <arg name="robot_name" value="$(arg robot_name)"/>-->
<!--        <arg name="robot_type" value="$(arg robot_type)"/>-->
<!--    </include>-->
    <include file="$(find spark_fast_lio)/launch/dcist/mapping_hamilton.launch" if="$(arg run_lidar_odom)">
       <arg name="robot_name" value="$(arg robot_name)"/>
   </include>
```

______________________________________________________________________

## Acknowledgement

Thanks for [HKU MaRS Lab](https://mars.hku.hk/) guys. The original code can be found [here](https://github.com/hku-mars/FAST_LIO)
