# Lane Following 

### To execute lanefollowing :

##### Start obstacle distance finder
```
ros2 run lanefollowing avoid
```
##### Follow these commands :
```
ros2 run lanefollowing stop.py
```
Then,
```
ros2 run twist_mux twist_mux --ros-args --params-file /home/tamoghna/ros2_ws/src/lanefollowing/src/config/twist_mux.yaml -r cmd_vel_out:=/vidhyut/cmd_vel
```
Modify the above line's param file location.
```
ros2 run lanefollowing switch.py
```
And, you should be done.

Subject to random errors tho. :-|