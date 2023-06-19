# Adaptive FAST/FASTER-LIO

+ Adaptively resizing the voxel resolution for being more robust on challenging environments (e.g., narrow stairs, aisles, etc.)
+ Original [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)


## Note - how to use `Adaptive mode`
+ Edit parameters in `.yaml` files

```yaml
adaptive:
    adaptive_voxelization_en: true # If true, ADA-FAST-LIO2, or just FAST-LIO2
    filter_size_surf_smaller: 0.3 # It should be smaller than `filter_size_surf`
    neighbor_xy_thres: 5.0 # xy neighbor threshold
    num_thr_adaptive_voxelization: 750
    num_thr_adaptive_voxelization_neighbor: 250
```


## How to build and use
+ Put the code in your workspace/src folder
```shell
catkin build -DCMAKE_BUILD_TYPE=Release
```
+ Then run
```shell
roslaunch ada_lio mapping_ouster.launch
roslaunch ada_lio mapping_velodyne.launch
roslaunch ada_lio mapping_livox.launch
```