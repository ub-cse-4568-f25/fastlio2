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

---

## ToDo 

- [ ] Set all the launch files for convenience
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

