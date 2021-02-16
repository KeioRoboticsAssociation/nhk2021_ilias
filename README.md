# nhk2021

**NHK Robocon 2021** project  team "**ilias**" (KeioRoboticsAssociation:robot:)

書きかけです



**cartographer_ros** **branch**

google cartographerを用いたマッピングと自己位置推定を画策するブランチ

だったがcartographerで作成したmapとamclの相性が悪そうだったのでいったん凍結

使用するとしたら自己位置推定もcartographerで行う



## Git管理上の注意

1. ”機能ごとに”ブランチを切ってください。名前は基本的には「featrue/機能名」としましょう
1. mainブランチには基本的には触らないようにしましょう。(管理者がmainにマージします)
2. mainにマージする時はPull Requestを作成してください。マージできるか確認したら管理人がマージします。
3. 他に管理上気になったことがあったら追記してください。



## 依存パッケージ

- 足回り

  https://github.com/KeioRoboticsAssociation/wheelctrl.git

- 自己位置推定

  https://github.com/cra-ros-pkg/robot_localization.git

  https://github.com/ros-planning/navigation.git

  https://github.com/yoshito-n-students/bno055_usb_stick.git

  https://github.com/ros-perception/laser_filters

- ROS<->mbedのシリアル通信

  https://github.com/moden3/serial_test.git

- 4輪メカナムのシミュレーション

  https://github.com/RBinsonB/nexus_4wd_mecanum_simulator.git

- Joy stick

  https://github.com/ros-drivers/joystick_drivers



## 導入

```shell
cd ~/catkin_ws/src
git clone https://github.com/KeioRoboticsAssociation/nhk2021.git
git clone https://github.com/KeioRoboticsAssociation/wheelctrl.git
git clone https://github.com/RBinsonB/nexus_4wd_mecanum_simulator.git
git clone https://github.com/cra-ros-pkg/robot_localization.git
git clone https://github.com/yoshito-n-students/bno055_usb_stick.git
git clone https://github.com/moden3/serial_test.git

sudo apt-get update
sudo apt-get install -y ros-melodic-navigation
sudo apt-get install -y ros-melodic-robot-localization
sudo apt-get install -y ros-melodic-laser-filters

cd ../
catkin_make
```



## 起動

- TRの起動

  ```shell
  roslaunch nhk2021_launcher control_TR.launch
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
