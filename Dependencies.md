# 実行依存パッケージ

各パッケージの実行依存パッケージを記載しています。

 desktop-full環境を前提としているため、それ以外の環境では他に足りないものがある可能性があります。

その場合は各パッケージごとに以下を実行することで、sudo apt -get installできるものは自動でインストールされます。

```shell
rosdep install -i --from-paths <PACKAGE_PATH> # <PACKAGE_PATH>にパッケージのパスを入れる
```

sudo apt -get installできないものに関しては愚直にgit cloneするか、もしくはvcstoolを用いて.rosinstallを参照することでインストールします。

```shell
sudo apt install python-vcstool
cd ~/catkin_ws
vcs import src < src/nhk2021_ilias/nhk2021_launcher/nhk2021_launcher.rosinstall
vcs import src < src/nhk2021_ilias/nhk2021_webgui/nhk2021_webgui.rosinstall
```



## bezier_path_planning_pursuit

特になし



## joy_commander

- joy, joystick_drivers

  ```shell
  sudo apt-get install -y ros-melodic-joy
  sudo apt-get install -y ros-melodic-joystick-drivers
  ```

  

## nhk2021_launcher

- 足回り

  - wheelctrl

    ```shell
    git clone https://github.com/KeioRoboticsAssociation/wheelctrl.git
    ```

- 自己位置推定

  - robot_localization

    ```shell
    sudo apt-get install -y ros-melodic-robot-localization
    ```

  - navigation (amcl, map_server)

    ```shell
    sudo apt-get install -y ros-melodic-navigation
    ```

  - bno055_usb_stick

    ```shell
    git clone https://github.com/yoshito-n-students/bno055_usb_stick.git
    git clone https://github.com/yoshito-n-students/bno055_usb_stick_msgs.git
    ```

  - laser_filters

    ```shell
    sudo apt-get install -y ros-melodic-laser-filters
    ```

- ROS<->mbedのシリアル通信

  - serial_test

    ```shell
    git clone https://github.com/moden3/serial_test.git
    ```

- joy, joystick_drivers

  ```shell
  sudo apt-get install ros-melodic-joy
  sudo apt-get install ros-melodic-joystick-drivers
  ```

  

## nhk2021_simulator

- gazebo_ros

  ```shell
  sudo apt install ros-melodic-gazebo-ros
  ```

- gazebo_ros_control 

  ```shell
  sudo apt install ros-melodic-gazebo-ros-control 
  ```

- ros_control

  ```shell
  sudo apt install ros-melodic-ros-control
  ```

- ros_controllers 

  ```shell
  sudo apt install ros-melodic-ros-controllers 
  ```

  

## nhk2021_webgui

- roswww

  ```shell
  git clone https://github.com/tork-a/roswww.git
  ```

  aptからもインストールできるが、うまく実行できなかった。

  git cloneしてrosdepで依存関係解決したほうが良い。

- rosbridge

  ```shell
  sudo apt install ros-melodic-rosbridge
  ```

  roswwwの依存関係を解決する際に自動でインストールされる

- tf2_web_republisher

  ```shell
  sudo apt install ros-melodic-tf2-web-republisher 
  ```

  