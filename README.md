# costumized by CYZ
for vertical test in 2d lidar.

# how to run
1. roslaunch voxblox_ros lidar2d_pt.launch
2. If using 2060 pc, go to cpc_ws4dataset. Otherwise, move ref_ugv to a source folder and build it.
3. roslaunch ref voxblox_ref.launch
4. rosbag play lidar_oak_pt.bag --clock