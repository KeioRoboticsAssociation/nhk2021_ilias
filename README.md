<img src="https://keiorogiken.files.wordpress.com/2018/12/e382abe383a9e383bc.png?w=2160" width="50%"/>

# nhk2021_ilias

**NHK Robocon 2021** project  team "**ilias**" (KeioRoboticsAssociation:robot:)



## Git管理上の注意

1. ”機能ごとに”ブランチを切ってください。名前は基本的には「featrue/機能名」としましょう
1. mainブランチには基本的には触らないようにしましょう。(管理者がmainにマージします)
2. mainにマージする時はPull Requestを作成してください。マージできるか確認したら管理人がマージします。
3. 他に管理上気になったことがあったら追記してください。



## Install

1. Clone this repositoty

```shell
cd ~/catkin_ws/src
git clone https://github.com/KeioRoboticsAssociation/nhk2021_ilias.git
```

2. Resolve dependencies (from git repository)

by **vcstool**

```shell
sudo apt install python-vcstool
cd ~/catkin_ws
vcs import src < src/nhk2021_ilias/nhk2021_launcher/nhk2021_launcher.rosinstall
vcs import src < src/nhk2021_ilias/nhk2021_webgui/nhk2021_webgui.rosinstall
```

or

by **git clone**

```shell
cd ~/catkin_ws/src
git clone https://github.com/KeioRoboticsAssociation/wheelctrl.git
git clone https://github.com/yoshito-n-students/bno055_usb_stick.git
git clone https://github.com/moden3/serial_test.git
git clone https://github.com/tork-a/roswww.git
```

3. Resolve dependencies (from apt repository)

```shell
cd ~/catkin_ws/src
rosdep install -i --from-paths .
```

4. Give permission to task_selector.py (in nhk2021_launcher pkg)

```shell
cd ~/catkin_ws/src/nhk2021_ilias
chmod +x ./nhk2021_launcher/scripts/task_selector.py
```

5. Build

```shell
cd ~/catkin_ws
catkin_make
```



A detail description about dependencies of this repository locates [Here](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/Dependencies.md).



## 各パッケージについて

- [bezier_path_planning_pursuit](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/bezier_path_planning_pursuit/README.md)

  経路計画、追従を担当するパッケージ

- [joy_commander](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/joy_commander/README.md)

  Joyコンで速度指令値を送れるパッケージ

- [nhk2021_launcher](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/nhk2021_launcher/README.md)

  ロボットを起動するlaunchファイルが入っているパッケージ

- [nhk2021_simulator](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/nhk2021_simulator/README.md)

  Gazeboによるシミュレーターを構築するためのファイルが入っているパッケージ

- [nhk2021_webgui](https://github.com/KeioRoboticsAssociation/nhk2021_ilias/blob/main/nhk2021_webgui/README.md)

  WebGUIを起動するためのファイルが入っているパッケージ



## Lisence

The applications are licensed under GPLv3 license.
