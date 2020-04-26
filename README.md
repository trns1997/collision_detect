# collision_detect

## Getting Started
```
cd PATH_TO_CATKIN_WS
git clone https://github.com/trns1997/collision_detect.git
caktin_make 
```
Make sure to source the catkin_ws. Check if the `collision_detect` package is properly setup by trying:
```
roscd collision_detect/
```

## Running the Nodes
Terminal 1
```
roslaunch collision_detect collision_detect.launch
```
Terminal 2
```
rosbag play PATH_TO_CATKIN_WS/src/collision_detect/bag project.bag
```
use `-r` flag to speed up the play speed if you please.

Terminal 3
```
rostopic echo /DistStatus
```
View the following video to see the steps and the result:
<img src= />
