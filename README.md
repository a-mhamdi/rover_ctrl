# Autonomous navigation of a mobile robot navigation using ROS2 - Humble Hawksbill

- Add this alias to your preferred shell startup script _(`zsh` in my case)_
    ```zsh
    echo "alias humble=\"source /opt/ros/humble/setup.zsh\"" >> ~/.zshrc
    ```
- Source `ROS2` installation
   ```zsh
    humble
    ```
- Check that the environment is properly set up 
   ```zsh
   printenv | grep -i ros
   ```
   _You should get an output as follows, along with other environment variables:_
  
   > ROS_DISTRO=humble
    
   > ROS_PYTHON_VERSION=3
   
   > ROS_VERSION=2

The official and detailed documentation is accessible through this [link](https://docs.ros.org/en/humble/index.html).

- Build the package using `colcon` tool
    ```zsh
    colcon build --symlink-install
    ```
- Source the local setup file
    ```zsh
    source install/local_setup.zsh
    ```
- Launch the `launch` file
    ```zsh
    ros2 launch rover_ctrl rover.launch.py
    ```
- The launch file `launch_sim.launch.py` displays the robot in `gazebo` and publishes the `/robot_description` and `/scan` topics to `rviz`
    ```zsh
    ros2 launch rover_ctrl launch_sim.launch.py
    ```
- Mapping using **SLAM** toolbox 
    ```zsh
    ros2 launch rover_ctrl launch_slam.launch.py
    ```

  [![DEMO SLAM](https://img.youtube.com/vi/I0WGgt07Tbg/0.jpg)](https://www.youtube.com/watch?v=I0WGgt07Tbg)
  
- Localization and navigation
    ```zsh
    ros2 launch rover_ctrl launch_nav.launch.py
    ```

  [![DEMO NAV2](https://img.youtube.com/vi/0SOzMY7_PX4/0.jpg)](https://www.youtube.com/watch?v=0SOzMY7_PX4)
