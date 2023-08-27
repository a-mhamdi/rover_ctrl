# ROVER CONTROL

## Autonomous Robot Navigation Using ROS2 - Humble Hawksbill

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
   
- Build the package using `colcon` tool
    ```zsh
    colcon build
    ```
- Source the local setup file
    ```zsh
    source install/local_setup.zsh
    ```
- Launch the `launch` file
    ```zsh
    ros2 launch rover_ctrl rover.launch.py
    ```
    
The official and detailed documentation is accessible through this [link](https://docs.ros.org/en/humble/index.html).
