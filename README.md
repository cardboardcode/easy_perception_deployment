## **What Is This?**

**epd_core** is a ROS2 package that accelerates the **training** and **deployment** of **Computer Vision** (CV) models for industries. Unlike **easy_perception_deployment**, **epd_core** aims to integrate contemporary Detectron2 as well as restructure the vision processing workflow into more modular components for better debugging and higher performance. 

Rather than provide a general tool kit for ROS 2 ready vision-processing, **epd_core** focuses on higher **accuracy** and **performance** of object detection and pose estimation. 

> [!NOTE]  
> To be dockerised for ease-of-use.

## **Build** :hammer:

```bash
cd $HOME
```

```bash
mkdir -p epd_ws/src && cd epd_ws/src
```

```bash
git clone https://github.com/cardboardcode/epd_core.git --single-branch --branch forward-dev --depth 1
```

```bash
cd $HOME/epd_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
rosdep install --from-paths src --ignore-src --rosdistro=humble -y
```

```bash
colcon build --symlink-install
```

## **Run** :arrow_forward:

```bash
cd $HOME/epd_ws
```

```bash
source install/setup.bash
```

```bash
ros2 launch epd_core run.launch.py
```
