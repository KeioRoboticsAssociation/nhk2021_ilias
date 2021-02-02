# joy_commander_tr

## これは何

ジョイコンでbase_linkへの速度指令値(/cmd_vel)を生成するパッケージ

特定のボタンを押したら矢を発射するなどは後日追加する



## Subscribed Topics

- **/joy** (type : `sensor_msgs::Joy`)



## Published Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])



## Launch

```shell
roslaunch joy_commander_tr joy_commander_tr.launch
```



## 補足

キーコンフィグは`./include/joy_commander_tr.h`の#defineの値を変更すれば容易に変更可能