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

`Fiesta<DepthMsgType, PoseMsgType>::Fiesta(ros::NodeHandle node)`

- ![Fiesta-ln88](./pic1/Fiesta-ln88.png)
- 设置参数
- 初始化esdf_map  [ESDFMap](#ESDFMap)
- ![Fiesta-ln102](./pic1/Fiesta-ln102.png)
- 设置概率参数
  - logit(p) = log(x / (1 - x))
- (array模式)初始化set_free\_, set\_occ\_ 储存空间, 赋值为0
- ![Fiesta-ln115](./pic1/Fiesta-ln115.png)
- subscribe topics
  - transform 位姿 [PoseCallback()](#PoseCallback)
  - depth 深度/点云 [DepthCallback()](#DepthCallback)
- publish topics
  - slice
  - occupancy
  - text
- 定时循环运行 [UpdateEsdfEvent()](#UpdateEsdfEvent)





`void Fiesta<DepthMsgType, PoseMsgType>::PoseCallback(const PoseMsgType &msg)`<a name=PoseCallback></a>

- ![Fiesta-ln441](./pic1/Fiesta-ln441.png)
- 把 (时间戳, 位置, 姿态) 加入transform_queue_
  - 后续的处理在[SynchronizationAndProcess()](#SynchronizationAndProcess)





`void Fiesta<DepthMsgType, PoseMsgType>::DepthCallback(const DepthMsgType &depth_map)`<a name=DepthCallback></a>

- 把 深度图 加入depth_queue_
- [SynchronizationAndProcess()](#SynchronizationAndProcess)





`void Fiesta<DepthMsgType, PoseMsgType>::SynchronizationAndProcess()`<a name=SynchronizationAndProcess></a>

当depth_queue_还有深度图时

- *指示flag* new_pos = false
- depth_time 深度队列里的第一个深度图的时间戳
- 找到transform_queue_里时间戳最接近的msg
  - if 时间戳 <= depth_time + ros::Duration(time_delay)
  - 记录位置和姿态, new_pos = True
  - 从队列里pop掉
  - 直到队列清空或时间戳大于 ...(上面时间)
- ![Fiesta-ln415](./pic1/Fiesta-ln413.png)
- transform_ 投影矩阵 深度图坐标系->机体->相机
- raycast_origin_  原点
- 如果是深度图
  - 每个点投影到相机(?)坐标系 形成 点云
  - 如果parameters\_.use\_depth\_filter\_
    - 忽略靠近图像边缘的点
    - 每个点重投影到上一张图像比较深度差距, 忽略大于阈值的点
- 如果是点云
  - 从ROSMsg转换成pcl点云
- 进行raycast [RaycastMultithread()](#RaycastMultithread)





`void Fiesta<DepthMsgType, PoseMsgType>::RaycastMultithread()`<a name=RaycastMultithread></a>

- parameters\_.ray\_cast\_num\_thread_==0
  - 单线程处理 [RaycastProcess()](#RaycastProcess)
- parameters\_.ray_cast_num_thread_ > 0
  - 多线程处理 [RaycastProcess()](#RaycastProcess)
  - 修改launch中的参数, 程序出错, 待续





`void Fiesta<DepthMsgType, PoseMsgType>::RaycastProcess(int i, int part, int tt)`<a name=RaycastProcess></a>

- ![Fiesta-ln204](./pic1/Fiesta-ln204.png)
- 对于点云里的每一个点:
  - 转换到世界坐标系
  - 如果超出ray最长距离, 截断到最长距离
    - 只考虑最长距离, 也就是到视线极限仍然是空
    - tmp_idx = esdf_map_->[SetOccupancy(pos, 0)](#SetOccupancy_pos) 设置为0 - free
  - tmp_idx = esdf_map_->[SetOccupancy(pos, 1)](#SetOccupancy_pos) 设置为1 - occupy
  - 设置set_occ_对应的tmp_idx
    - (HASH_TABLE) `set_occ_.insert(tmp_idx)`
    - (ARRAY) `set_occ_[tmp_idx] = tt`
  - ![ESDFMap-ln423](./pic1/Fiesta-ln233.png)
  - output是光线经过的所有格子的坐标
  - ![Fiesta-ln239](./pic1/Fiesta-ln239.png)
  - 对于output中的每一个格子坐标(int), 从最远的开始:
    - tmp 计算格子中心点坐标(double)
    - length 到光心的距离
      - 用于判断点是否落在min_ray_length和max_ray_length范围内
    - `tmp_idx = esdf_map_->SetOccupancy(tmp, 0)` 该处occupancy状态设置为0
    - 如果tmp_idx已经在set_free_存在
      - 说明后面坐标都已经raycast过了
      - break





`void Fiesta<DepthMsgType, PoseMsgType>::UpdateEsdfEvent(const ros::TimerEvent & /*event*/)`<a name=UpdateEsdfEvent></a>







## src/ESDFMap.cpp

`fiesta::ESDFMap::ESDFMap(...)`<a name=ESDFMap></a>

array模式

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

HASH_TABLE模式

- ***待续***





`int fiesta::ESDFMap::SetOccupancy(Eigen::Vector3d pos, int occ)`<a name=SetOccupancy_pos></a>

- `Pos2Vox(pos, vox)` pos转换成vox
- return [SetOccupancy(vox, occ)](#SetOccupancy_vox)



`int fiesta::ESDFMap::SetOccupancy(Eigen::Vector3i vox, int occ)` <a name=SetOccupancy_vox></a>

- `idx = Vox2Idx(vox)` vox转换成idx
- 如果vox在地图内
- ![ESDFMap-ln423](./pic1/ESDFMap-ln423.png)
  - 只要光线穿过num_miss_就加一
  - num_hit_ 加上occ(0-free, 1-occupy)
- 只被光线穿过一次的
  - （vox, 0.0）加入occupancy_queue_队列
    - 后续的处理在 [UpdateOccupancy()](#UpdateOccupancy)



`bool fiesta::ESDFMap::UpdateOccupancy(bool global_map)`<a name=UpdateOccupancy></a>

