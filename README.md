# Starbot Delivery Final Masterclass Project

## Install a Few Dependencies
```bash
pip install numpy==1.24.3 
pip install ultralytics
```

---

<details>
  <summary><h2><strong>Simulation Robot (Click to expand)</strong></h2></summary>

### Launch the Simulation World
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
```

### Launch the MoveIt Node Group
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_moveit_config move_group.launch.py
```

### Launch the MoveIt Rviz Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_moveit_config moveit_rviz.launch.py
```

### Launch the Planning Scene
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_trajectory_manipulation planning_scene.launch.py
```

### Launch the Perception Vision System (YOLOv8)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hole_detection_yolo hole_detector.launch.py
```

### Launch the Webpage
#### ROSBridge WebSocket
```bash
cd ~/ros2_ws/src/webapp
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### HTTP Server for the Webpage
```bash
cd ~/ros2_ws/src/webapp
python3 -m http.server 7000
```

#### Retrieve Webpage and ROSBridge Addresses
```bash
cd ~/ros2_ws/src/webapp
webpage_address
```
```bash
cd ~/ros2_ws/src/webapp
rosbridge_address
```

### Launch the Manipulation System (Coffee Order Handler)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_trajectory_manipulation coffee_order_handler.launch.py
```

</details>

---

<details>
  <summary><h2><strong>Real Robot (Click to expand)</strong></h2></summary>

### Launch the MoveIt Node Group
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch real_moveit_config move_group.launch.py
```

### Launch the MoveIt Rviz Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch real_moveit_config moveit_rviz.launch.py
```

### Launch the Planning Scene
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_trajectory_manipulation planning_scene_real.launch.py
```

### Launch the Perception Vision System (YOLOv8)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hole_detection_real_yolo hole_detector_real.launch.py
```

### Launch the Webpage
#### ROSBridge WebSocket
```bash
cd ~/ros2_ws/src/webapp
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### HTTP Server for the Webpage
```bash
cd ~/ros2_ws/src/webapp
python3 -m http.server 7000
```

#### Retrieve Webpage and ROSBridge Addresses
```bash
cd ~/ros2_ws/src/webapp
webpage_address
```
```bash
cd ~/ros2_ws/src/webapp
rosbridge_address
```

### Launch the Manipulation System (Coffee Order Handler for Real Robot)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_trajectory_manipulation coffee_order_handler_real.launch.py
```

</details>

---

## Notes

- Ensure that the `ros2_ws` workspace is properly built and sourced before running any commands.
- Replace any placeholder paths or addresses as necessary for your setup.
- Monitor the terminal outputs for errors or warnings and troubleshoot accordingly.
- For the webpage, make sure the HTTP server is running and accessible via the provided address.

---

This README ensures consistency and clarity when managing the simulation and real robot systems for the coffee ordering process.








