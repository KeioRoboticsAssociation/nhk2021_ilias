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
float32 position_x
float32 position_y
float32 position_theta
---
#feedback
float32 reference_point
float32 progress
float32 vx
float32 vy
float32 omega
```

- Goal
  - pathmode : パスの番号
  - direction : 進行方向 (1ならば順方向、0ならば逆方向)
- Feedback
  - reference_point : 現在プランナーがどのwaypointを参照しているか(waypointの間も参照するためfloat値)
  - progress : 進捗(%)
  - vx : x方向の速度指令値
  - vy : y方向の速度指令値
  - omega : yaw方向の速度指令値
- result
  - result : Goalにたどり着けたらTrue, そうでなければFalse (Statusで確認できるので、使うことはないだろう)
  - position_x : 到着した点のx座標
  - position_y : 到着した点のy座標
  - position_theta : 到着した点でのロボット角度



## Launch parameters

- **control_frequency** : publish frequency (default : 30[Hz])

- **base_frame_id** : ロボット座標系 (default : "base_footprint")

- **global_frame_id** : 固定座標系 (default : "map")

- **use_tf** : (default : "false")

  現在位置をglobal_frame_id->base_frame_idの位置関係から取得するかどうか

  trueにするのは、amclなどの自己位置推定ノードによってtfが補正されている場合

- **data_path** : csvファイルの格納されているディレクトリ (default : "`$(find nhk2021_launcher)/config/waypoints/TR`")

- **acc_lim_xy** : maximum value of xy accelaration (default : 2.5 [m/s^2])

- **max_vel_xy** : maximum value of xy velocity (default : 1.5 [m/s])

- **acc_lim_theta** : maximum value of theta accelaration (default : 3.2 [rad/s^2])

- **max_vel_theta** : maximum value of theta velocity (default : 1.57 [rad/s])

- **initial_vel** : 初速度 (default : 0.1 [m/s])

- **xy_goal_tolerance** :  (default : 0.05 [m])

  xy方向のゴールの精度。0.0とするとゴール座標と一致するまで進むが、多少余裕を持たせるとよいと思う。

- **yaw_goal_tolerance** :  (default : 0.01 [rad])

  yaw方向のゴールの精度。

- **angle_source** : (default : "pose")

  ロボットの車体角度の取得源を"**pose**", "**imu**", "**odom**"の3択から選択する。

  poseの場合は`geometry_msgs::PoseStamped`型のtopicより

  imuの場合は`sensor_msgs::Imu`型のtopicより

  odomの場合は`nav_msgs::Odometry`型のtopicより取得する

  pose, imu, odom以外の文字列を入れると強制終了するようにしている。

  

### Advanced settings

- **fix_angle_gain** :  (default : 3.0)

  このプランナーは目的地到達後に角度調整を行う。

  その際、角速度を$ω=Δθ\times fix\_angle\_gain$で定めている。

- **path_granularity** : (default : 0.01 [m])

  ​	本プランナーはパスに一定間隔で目標速度の情報を埋め込んでおり、その間隔をいくらにするかをこのパラメータで指定する。

  ​	基本変えなくてよい。

- **corner_speed_rate** :  (default : 0.8)

  コーナーでどのくらい攻めるか

  ロボットが立方体であると仮定した場合、corner_speed_rate = 1.0のとき遠心力と重力のモーメントが釣り合うまで速度を上げることを許容する。よって1.0未満であることが望ましい。



## Subscribed Topics

- **/odom** (type : `nav_msgs::Odometry`) (**use_tf = false の場合**)
- **/pose** (type : `geometry_msgs::PoseStamped`) (**angle_source = "pose" の場合**)
- **/imu** (type : `sensor_msgs::Imu`) (**angle_source = "imu" の場合**)



## Published Topics

- **/cmd_vel** (type : `geometry_msgs::Twist`)
- **/path** (type : `nav_msgs::Path`) (RVizでの可視化用)



## 補足

### waypointを格納するcsvの記述フォーマット

*1.csv*

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

- csvの名前は何でもよいが、上から読んでいくので1.csv, 2.csv...などとしていくとよい。

- csvの格納されているディレクトリはpath_planning_pursuitノードを起動する際に**data_path**パラメータに指定する



## 簡単な動作確認方法

このパッケージのノードを起動したのち、以下のコマンドを打つとaction client guiが立ち上がり、action serverとしての機能を容易に確認することができる。

```shell
rosrun actionlib axclient.py /bezier_path_planning_pursuit
```

