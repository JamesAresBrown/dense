# dense
## 嵌入VINS-Fusion进行稠密建图
在VINS-Fusion/路径下下载
启动 KITTI GPS Fusion (Stereo + GPS) 与 dense

### 4.2 KITTI GPS Fusion (Stereo + GPS)
Download [KITTI raw dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php) to YOUR_DATASET_FOLDER. Take [2011_10_03_drive_0027_synced](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip) for example.
Open three terminals, run vins, global fusion and rviz respectively.
Green path is VIO odometry; blue path is odometry under GPS global fusion.
```
    roslaunch vins vins_rviz.launch
    rosrun vins kitti_gps_test ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/ 
    rosrun global_fusion global_fusion_node
    rosrun dense dense_node
```
/config/dense1.yaml是默认配置文件，在文件中你可以配置生成路径与pmvs选项，pmvs选项的配置参见PMVS文档（一般不用修改）。
