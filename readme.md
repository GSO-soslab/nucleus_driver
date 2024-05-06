# Nucleus ROS 1 Driver

### Services
```
/nucleus_node/command
/nucleus_node/connect_serial
/nucleus_node/connect_tcp
/nucleus_node/disconnect
/nucleus_node/field_calibration
/nucleus_node/start
/nucleus_node/stop
```

### Topics

Original (Custom message types)

```
/nucleus_node/ahrs_packets
/nucleus_node/altimeter_packets
/nucleus_node/bottom_track_packets
/nucleus_node/current_profile_packets
/nucleus_node/field_calibration_packets
/nucleus_node/imu_packets
/nucleus_node/magnetometer_packets
/nucleus_node/water_track_packets
```

New (Standard ROS message types)

```
/nucleus_node/altitude_common                       [std_msgs/Float32]
/nucleus_node/bottom_lock_velocity_common           [geometry_msgs/Vector3]
/nucleus_node/imu_common                            [sensor_msgs/Imu]
/nucleus_node/magnetic_common                       [sensor_msgs/MagneticField]
/nucleus_node/pressure_common                       [sensor_msgs/FluidPressure]
```

## Instructions

1. Start nucleus node
    ```
    rosrun nucleus_driver_ros2 nucleus_node.py
    ```

2. Connect to nucleus (Currently `nucleus_node.py` automatically connects to the predefined ip)
    ```
    rosservice call /nucleus_node/connect_tcp "host: '$IP'"
    ```

    `$IP` is the ip address of the nucleus

3. Start Sensors
    ```
    rosservice call /nucleus_node/start 
    ```

4. Topics are now being published

5. Stop Sensors
    ```
    rosservice call /nucleus_node/stop 
    ```

6. Sending Commands
    ```
    rosservice call /nucleus_node/command 'COMMAND'
    ```
    `'COMMAND'` = `'SETAHRS,DS="ON"'`