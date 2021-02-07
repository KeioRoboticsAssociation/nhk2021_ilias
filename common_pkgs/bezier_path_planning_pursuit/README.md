# bezier_path_planning_pursuit

## これは何

経路計画、追従を行うROSパッケージ。

このROSパッケージが持つ機能は

1. 経路のwaypointを記述したcsvファイルを読み込み、waypointをベジェ曲線で補完したうえでロボットの速度、加速度制限を考慮した速度テーブルを作成する(経路計画)
2. 追従させたいPathの番号と方向を指定すると、そのPathに追従するような速度指令値をpublishする(経路追従)

2.の機能はactionlibを用いて実装しており、このROSノードはaction serverとして機能する。外部のaction clientからPathの番号と方向をGoalとして指定すると追従が開始し、追従が終了するとSUCCEEDEDステータスを返す。

なお、経路追従はpure pursuitアルゴリズムによって行われる。



## action msg

```yaml
#goal definition
int32 pathmode
int32 direction
---
#result definition
bool result
---
#feedback
float32 reference_point
```

- Goal
  - pathmode : パスの番号
  - direction : 進行方向 (1ならば順方向、0ならば逆方向)
- Feedback
  - reference_point : 現在プランナーがどのwaypointを参照しているか(waypointの間も参照するためfloat値)
- result
  - result : Goalにたどり着けたらTrue, そうでなければFalse (Statusで確認できるので、使うことはないだろう)



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])

- **zone** : 赤陣営("red")か青陣営("blue")かを与える (default : "blue")

- **use_odom_tf** : (default : "false")

  現在位置をodom tfから取得するかどうか

  trueならばodom tfから取得する

  falseならばodomトピックから取得する

  trueにするのは、amclなどの自己位置推定によってodom tfが補正されている場合

- **data_path** : csvファイルの格納されているディレクトリ (default : "`$(find pkg_manager_tr)/config/waypoints`")

- **acc_lim_xy** : maximum value of xy accelaration (default : 2.5 [m/s^2])

- **max_vel_xy** : maximum value of xy velocity (default : 1.5 [m/s])

- **acc_lim_theta** : maximum value of theta accelaration (default : 3.2 [rad/s^2])

- **max_vel_theta** : maximum value of theta velocity (default : 1.57 [rad/s])

- **initial_vel** : 初速度 (default : `max_accel / control_frequency` [m/s])

- **corner_speed_rate** :  (default : 0.8)

  コーナーでどのくらい攻めるか

  ロボットが立方体であると仮定した場合、corner_speed_rate = 1.0のとき遠心力と重力のモーメントが釣り合うまで速度を上げることを許容する。よって1.0未満であることが望ましい。

- **global_frame_id** :  Pathをどのフレームに描画するか (default : "odom")

- **xy_goal_tolerance** :  (default : 0.05)

  xy方向のゴールの精度。0.0とするとゴール座標と一致するまで進むが、多少余裕を持たせるとよいと思う。

  単位は無次元(厳密には、この数字の単位はwaypointの番号と同次元であり、waypointの間隔により長さは変化する)
  
- **yaw_goal_tolerance** :  (default : 0.01 [rad])

  yaw方向のゴールの精度。

- **angle_source** : (default : "pose")

  ロボットの車体角度の取得源を"**pose**", "**imu**"の2択から選択する。

  **実機**の場合はpose (bno055のposeトピックより取得)

  **シミュレーション**の場合は**imu** (imuのgazeboプラグインが出すimuトピックより取得)

  pose, imu以外の文字列を入れると強制終了するようにしている。



## Subscribed Topics

- **/pose** (type : `geometry_msgs::PoseStamped`) (**angle_source = "pose" の場合**)
- **/imu** (type : `sensor_msgs::Imu`) (**angle_source = "imu" の場合**)
- **/odom** (type : `nav_msgs::Odometry`) (**use_odom_tf = false の場合**)



## Published Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)
- **/path** (type : `nav_msgs::Path`) (RVizでの可視化用)



## 補足

### waypointを格納するcsvの記述フォーマット

*path_point_blue1.csv*

```csv
x,y,theta
0,0,0
0,2000,-
500,4000,-
0,8000,-
0,10000,0
```

- 上記のようにx, y, theta座標を記入する。単位は**mm**および**deg**。

- thetaを"-"とすると、その点は線形補間される。

- csvの名前はpath_point_"zone_name""path_number".csvとする。

- csvの格納されているディレクトリはpath_planning_pursuitノードを起動する際に**data_path**パラメータに指定する



また、新たにパスを追加、削除した際は`bezier_path_planning_pursuit/src/path_planner.h`の`#define LINE_NUM`の値を変更すること。例えば、csvを編集してpathが5つになった場合、以下のように編集する。

```cpp
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "path_planning.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/server/simple_action_server.h>          // action Library Header File
#include <bezier_path_planning_pursuit/PursuitPathAction.h> // PursuitPathAction Action File Header

using namespace bezier_path_planning_pursuit;

#define LINE_NUM 5 // <- modified

class Path_Planner
```



## 簡単な動作確認方法

このパッケージのノードを起動したのち、以下のコマンドを打つとaction client guiが立ち上がり、action serverとしての機能を容易に確認することができる。

```shell
rosrun actionlib axclient.py /path_planning_pursuit
```

