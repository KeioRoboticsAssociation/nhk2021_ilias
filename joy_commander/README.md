# joy_commander

## これは何

ジョイコンでロボットへの速度指令値(/cmd_vel)を生成するパッケージ


## Subscribed Topics

- **/joy** (type : `sensor_msgs::Joy`)

- **/teleopflag** (type : `std_msgs::Bool`)

  /teleopflagがtrueのときは/cmd_velのpublishをONに、falseのときはOFFにする

  外部ノードからpublishの可否を制御する用途に使用可能



## Published Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])
- **acc_lim_xy** : maximum value of xy accelaration (default : 2.5 [m/s^2])
- **max_vel_xy** : maximum value of xy velocity (default : 1.5 [m/s])

- **acc_lim_theta** : maximum value of theta accelaration (default : 3.2 [rad/s^2])

- **max_vel_theta** : maximum value of theta velocity (default : 1.57 [rad/s])



## 操作方法

キーコンフィグについては今後追記

コードを読んで感じてください(適当)



## 補足

キーコンフィグは`./include/joy_commander/joy_commander.h`の#defineの値を変更すれば変更可能