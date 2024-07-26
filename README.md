# FAST-LIO2 for Kimera-Multi Dataset 

+ Original [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)


## How to build and use
+ Put the code in your workspace/src folder
```shell
catkin build -DCMAKE_BUILD_TYPE=Release
```
+ Then run

```shell
roslaunch spark_fast_lio mapping_ouster.launch
roslaunch spark_fast_lio mapping_velodyne.launch
roslaunch spark_fast_lio mapping_livox.launch
```

Especially, in the Kimera-Multi dataset,

```
roslaunch spark_fast_lio mapping_${ROBOT_NAME}.launch save_dir:=${DIRECTORY} sequence_name:="${DATE}_{ROBOT_NAME}" robot_name:="${ROBOT_NAME}" 
// e.g.,
roslaunch spark_fast_lio mapping_sobek.launch save_dir:=${DIRECTORY} sequence_name:="12_08_sobek" robot_name:="sobek" 
```

Q. Why do we need to specify the robot name?
Because in `10_14` datatset, those are all recorded by `acl_jackal2`. 
For this reason, just in case, I separated and parameterized the sequence_name and robot_name.

---

## ToDo 

- [X] Set all the launch files for convenience
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

