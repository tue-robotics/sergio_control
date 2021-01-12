# sergio_control

[![CI](https://github.com/tue-robotics/sergio_control/workflows/CI/badge.svg)](https://github.com/tue-robotics/sergio_control/actions)

Control SERGIO with ROS Control

## Test

### Launch

#### test_transmission_manager

Test the transmission manager that parses a joint actuator transmission.

```
rostest sergio_control test_transmission_manager.test

```

#### test_single_joint_position_controller

Test a single joint position controller (simple motor / encoder set-up)

```
rostest sergio_control test_single_joint_position_controller.test

```

### Nodes

#### test_actuator

Test a single actuator by applying a force to the actuator and reading back the state.

```
rosrun sergio_control test_actuator
```

##### Parameters

- `~ethercat_interface`: Ethercat interface name (defaults to eno1)
- `~motor_slave`: Motor slave index (defaults to 1)
- `~motor_channel`: Motor channel index (defaults to 0)
- `~motor_volt_per_newton_meter`: Volts per newton meter, depends on amplifier (defaults to 22.801652754998294)
- `~encoder_slave`: Encoder slave index (defaults to 2)
- `~encoder_channel`: Encoder channel index (defaults to 0)
- `~encoder_counts_per_revolution`: Number of counts per revolution, take the quadrature into account, so x4 if every flank is count! (defaults to 1024)
- `~mechanical_reduction`: gear box ratio (defaults to 66.2204081633)
- `~command`: Force that should be applied to the actuator (defaults to 0.05)
- `~rate`: Update rate (defaults to 500)

#### test_hardware_interface

Hardware interface and ROS controller manager ([ROS Control](http://wiki.ros.org/ros_control)) with an ethercat_interface.

##### Parameters

- `~ethercat_interface`: Ethercat interface name (defaults to eno1)
- `~ethercat_actuators`: Map of ethercat actuators (the urdf should point to these actuators with transmission). Example map (required):

```
actuator1:
  encoder:
    slave: 2
    channel: 0
    encoder_counts_per_revolution: 2000 # 500 impulses but we count every transition, so * 4 = 2000
  motor:
    slave: 1
    channel: 0
    volt_per_newton_meter: 6.41 # V/Nm = 1 / (motor_constant / amplifier_gain)
actuator2:
  encoder:
    slave: 1
    channel: 1
    encoder_counts_per_revolution: 2000
  motor:
    slave: 1
    channel: 0
    volt_per_newton_meter: 6.41
```

- `~rate`: Update rate (defaults to 500)
- `/robot_description`: Robot description [urdf](http://wiki.ros.org/urdf)
