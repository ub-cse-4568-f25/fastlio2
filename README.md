# FAST-LIO2 for Kimera-Multi Dataset and DCIST Project

+ Original: [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)


## How to build


Put the code in your workspace/src folder

```shell
cd ${YOUR_WORKSPACE}/src
git clone git@github.mit.edu:SPARK/spark_fast_lio.git
cd spark_fast_lio
git submodule update --init --recursive
cd ../..
catkin build -DCMAKE_BUILD_TYPE=Release
```

## How to use

+ Then run

```shell
roslaunch spark_fast_lio mapping_ouster.launch
roslaunch spark_fast_lio mapping_velodyne.launch
roslaunch spark_fast_lio mapping_livox.launch
```

Especially, in the **Kimera-Multi** dataset,

```
roslaunch spark_fast_lio mapping_${ROBOT_NAME}.launch save_dir:=${DIRECTORY} sequence_name:="${DATE}_{ROBOT_NAME}" robot_name:="${ROBOT_NAME}" 
// e.g.,
roslaunch spark_fast_lio mapping_sobek.launch save_dir:=${DIRECTORY} sequence_name:="12_08_sobek" robot_name:="sobek" 
```

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

---

## ToDo 

- [X] Set all the launch files for convenience
- [X] Support DCIST project
- [ ] Remove adaptive mode part (due to patent issue from KAIST)

---

## Note - how to use `Adaptive mode` (yet will be deprecated soon)

+ Edit parameters in `.yaml` files

```yaml
filter_size_map: 0.5 #bigger voxel size
adaptive:
    adaptive_voxelization_en: true # If true, ADA-FAST-LIO2, or just FAST-LIO2
    filter_size_map_smaller: 0.2 # It should be smaller than `filter_size_map`
    neighbor_xy_thres: 5.0 # xy neighbor threshold
    num_thr_adaptive_voxelization: 700
    num_thr_adaptive_voxelization_neighbor: 300 # For Velodyne 16, this method is not applicable
```

