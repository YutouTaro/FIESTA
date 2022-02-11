## global variables
```c++
  reserve_size_ // 预留空间大小, 必需
  occupancy_buffer_ // 概率化的occupancy, 必需
  distance_buffer_ // SDF距离, 可以替换成squared_distance方便计算和比较
  closest_obstacle_ // 坐标(point)对应的idx的最近障碍物的int坐标, 是否可以用当前点指向障碍物的向量代替(待研究)
  vox_buffer_ // 体素坐标
  num_hit_.resize(reserve_size_);
  num_miss_.resize(reserve_size_);
```

## TO NOTICE
不一定要insert_queue_和delete_queue_全加入updae_queue_才计算neighbour的sdf

先不计算delete_queue的sdf, 等insert_queue全传播完了还有没处理的再处理

