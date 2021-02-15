# nhk2021

**NHK Robocon 2021** project  team "**ilias**" (KeioRoboticsAssociation:robot:)



## Git管理上の注意

1. ”機能ごとに”ブランチを切ってください。名前は「featrue/機能名」とします
1. mainブランチには基本的には触らないようにしましょう。(管理者がmainにマージします)
1. mainにマージする時はプルリクを管理者に送ってください。マージできるか確認したら許可を出すのでマージを行ってください。Slackで報告してくれればさらにgood。
1. 他に管理上気になったことがあったら追記してください。



## 構成

- **TR** - Throwing Robot に関するメタパッケージ
- **DR** - Defensive Robot に関するメタパッケージ
- **common pkg** - 共通するパッケージ
- **Simulation** - Simulation on Gazebo



## 依存パッケージ

- 足回り

  https://github.com/KeioRoboticsAssociation/wheelctrl_4ws.git

- 自己位置推定

  https://github.com/cra-ros-pkg/robot_localization.git

  https://github.com/ros-planning/navigation.git　(melodic-devel)

  https://github.com/yoshito-n-students/bno055_usb_stick.git

- ROS<->mbedのシリアル通信

  https://github.com/moden3/serial_test.git

- 4輪メカナムのシミュレーション

  https://github.com/RBinsonB/nexus_4wd_mecanum_simulator.git



## 導入

```shell
cd ~/catkin_ws/src
git clone https://github.com/KeioRoboticsAssociation/nhk2021.git
git clone https://github.com/KeioRoboticsAssociation/wheelctrl_4ws.git
git clone https://github.com/RBinsonB/nexus_4wd_mecanum_simulator.git
git clone https://github.com/cra-ros-pkg/robot_localization.git
git clone https://github.com/yoshito-n-students/bno055_usb_stick.git
git clone https://github.com/moden3/serial_test.git

sudo apt-get update
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-laser-filters
sudo apt-get install -y ros-melodic-gmapping

rosrun map_server map_saver _map:=nhk2021map
cd ../
catkin_make
```



## 起動

- TRの起動

  ```shell
  roslaunch pkg_manager_tr activate_TR.launch
  ```

- simulatorの起動

  ```shell
  roslaunch nhk2021_simulator 4wd_mecanum_simulation.launch
  ```

  詳しくはsimulatorのREADMEを参照



## Trouble Shooting

- bno055が起動できない

  ポートに実行権限を与える

  ```shell
  sudo chmod 666 /dev/ttyACM0
  ```



## Lisence

The applications are licensed under GPLv3 license.
