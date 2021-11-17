# test_fiesta.cpp阅读笔记

```bash
# 每个新窗口记得source ros和catkin_ws
source /opt/ros/melodic/setup.bash && source /home/yutou/catkin_ws/devel/setup.bash
# 窗口1 运行主体
roslaunch fiesta cow_and_lady.launch
# 窗口2 运行rosbag
rosbag play ./rosbag/data.bag

```



\* `HASH_TABLE` is disabled in `include/parameters.h`  



## `include/Fiesta.h`

- 设置参数
- 初始化esdf_map  [ESDFMap](#ESDFMap)
- 







## src/ESDFMap.cpp

### ESDFMap()<a name=ESDFMap></a>

array版

- 输入: Eigen::Vector3d origin 中心, double resolution_ 分辨率, Eigen::Vector3d map_size 地图大小
- ![ESDFMap-ln171](./pic1/ESDFMap-ln171.png)
  - 计算grid_size
  - grid_size_yz_ 用途待定
  - SetOriginalRange()
  - ![ESDFMap-ln812](./pic1/ESDFMap-ln812.png)
    - ​	初始化最小, 最大vec 和 上一个最小, 最大vec
- ![ESDFMap-ln188](./pic1/ESDFMap-ln188.png)
  - 初始化并赋值
    - occupancy_buffer_	<-0
    - 距离distance_buffer_ <-undefined_
    - 最近障碍物坐标closest_obstacle_ <-undefined\_, undefined\_, undefined\_
    - num_hit\_, num_miss_ <-undefined_
    - head_ 多一个size <-undefined_
    - prev\_, next\_ <-undefined_

HASH_TABLE版

- ***待续***
