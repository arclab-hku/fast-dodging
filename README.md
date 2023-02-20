# Fast-dodging

In this repo, we provide the code of our fast detection and state estimation of fast moving small objects. Related paper: "Perception and Avoidance of Multiple Fast Moving Small Objects forQuadrotors with Only Low-cost RGBD Camera". 

youtube link: https://www.youtube.com/watch?v=n1rlnFTOBGE

bilibili link: https://www.bilibili.com/video/BV1uT4y1Q7zx/?spm_id_from=333.337.search-card.all.click

# Yolo-Fastest + 3D-SORT

## ncnn compile
refer to https://github.com/Tencent/ncnn,
copy the contents below ncnn/build/install/ to yolo_fastest_ros/ncnn/

!!! pay attention that the different compiling steps between x86 and arm CPU

## run
```
roslaunch d435i.launch ## your own sensor startup file 
roslaunch yolo_detector yolo_ros.launch
roslaunch sort_ros sort_ros.launch
```

realized multi-object detection, 3d tracking and trajectory prediction.

