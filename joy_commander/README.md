# joy_commander

## これは何

ジョイコンでbase_linkへの速度指令値(/cmd_vel)を生成するパッケージ


## Subscribed Topics

- **/joy** (type : `sensor_msgs::Joy`)



## Published Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])



## Launch

```shell
roslaunch joy_commander joy_commander.launch
```



## 補足

キーコンフィグは`./include/joy_commander/joy_commander.h`の#defineの値を変更すれば変更可能