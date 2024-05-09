1. dvrk console:  
```ros2 run dvrk_robot dvrk_console_json -j ~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json```

2. dvrk state publishers:  
```ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1```

3. RViz2:  
```ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz```

to run all of the above commands at the same time:  
```ros2 run dvrk_robot dvrk_console_json -j ~/dvrk2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json & ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1 & ros2 run rviz2 rviz2 -d ~/dvrk2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz &```

4. start marker node:  
```ros2 run ros2_course dummy_marker```

5. start grasping trajectory:  
```ros2 run ros2_course psm_grasp```
