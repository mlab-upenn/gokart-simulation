# simulator


## Usage

```text
$ ros2 launch simulator simulation.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'config':
        Path to the parameters file that will be added to all nodes (except Gazebo ones). Must be either an absolute path or a relative path within the simulator package's share dir.
        (default: 'config/simulation.yaml')

    'world':
        Path to the world model file to load. Must be either an absolute path or a relative path within the simulator package's share dir.
        (default: 'worlds/test_world.world')

    'urdf_model':
        Path to the robot URDF file (xacro supported). Must be either an absolute path or a relative path within the simulator package's share dir.
        (default: 'urdf/gokart/main.urdf')

    'rviz_config':
        Path to the RViz config file to use. Must be either an absolute path or a relative path within the simulator package's share dir.
        (default: 'rviz/demo.rviz')

    'headless':
        If true, no GUI tools (gzclient, rviz, teleop) are started.
        (default: 'False')

    'start_gzclient':
        Whether to start Gazebo GUI (gzclient).
        (default: 'true')

    'start_rviz':
        Whether to start RViz.
        (default: 'False')

    'start_joint_state_publisher_legacy':
        Whether to start joint_state_publisher. Not needed anymore. simulator_gazebo_plugin publishes joint states by default (at the correct frequency). Do NOT this use unless you set simulator_gazebo_plugin publishJointStates to false.
        (default: 'false')

    'teleop':
        Start teleop tool. Allowed values: key, joy, none.
        (default: 'none')
```

### Examples

* key teleop
  ```bash
  ros2 launch simulator simulation.launch.py teleop:=key
  ```
* **key teleop and RViz**
  ```bash
  ros2 launch simulator simulation.launch.py teleop:=key start_rviz:=true
  ```


## Useful resources

* https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/
* https://drive.google.com/drive/folders/1VrgqtETVSuheNITop0MXDy4BY3U8IphQ
* https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
* https://drive.google.com/drive/folders/1PrEavKxQoaQIPLqHwejzreO8XKaX80hW
