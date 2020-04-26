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
*Terminal 1*
```
roslaunch collision_detect collision_detect.launch
```
*Terminal 2*
```
rosbag play PATH_TO_CATKIN_WS/src/collision_detect/bag project.bag
```
use `-r` flag to speed up the play speed if you please.

*Terminal 3*
```
rostopic echo /DistStatus
```
View the following [video](https://github.com/trns1997/collision_detect/blob/master/media/demoVideo.mp4) to see the steps and the result. But heres the gif version if you are lazy to click the vid:
<img src= https://github.com/trns1997/collision_detect/blob/master/media/demoVid.gif/>

## Under the Hood

### Nodes and Topics
```
rosrun rqt_graph rqt_graph
```
<img src=https://github.com/trns1997/collision_detect/blob/master/media/rqt_graph.png>

### Ros Service Structure
```srv/GetDist.srv```
```
float32 carEnu_x
float32 carEnu_y
float32 carEnu_z
float32 obsEnu_x
float32 obsEnu_y
float32 obsEnu_z
---
float32 dist  
```
The service takes **3D Enu coordinates** of the **Car** and the **Obstacle** and as response outputs the **Distance** between the 2 objects.

### Ros Message Structure
```msg/collisionInfo.msg```
```
float32 dist
string flag
```
The custom messasge comprises of the **Distance** between the **Car** and **Obstacle**, And a **Status Flag** that categorizes the distance between the 2 objects as either *Safe* or *Unsafe* or *Crash*.

### Dynamic Reconfigure Gui
<img src=https://github.com/trns1997/collision_detect/blob/master/media/reconf_gui.png>
This gui allows you to change the **Min Safety Distace** and the **Max Crash Distance** between the Car and Obstacles.

### Tf Tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
<img src=https://github.com/trns1997/collision_detect/blob/master/media/tf_tree.png>
