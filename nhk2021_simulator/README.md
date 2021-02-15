# NHK2021_simulator
## これは何

NHK2021の環境をgazeboで再現するパッケージ。



## 依存パッケージ

- https://github.com/RBinsonB/nexus_4wd_mecanum_simulator

  4輪メカナムの足回りシミュレーションに用いる



## ロボットモデル

現在は以下の2つに対応

- 簡易的な差動2輪モデル
- 簡易的な4輪メカナムモデル



## Launch

- フィールド+簡易的な差動2輪モデル

  ```shell
  roslaunch nhk2021_simulator diff_drive_simulation.launch
  ```

- フィールド+簡易的な4輪メカナムモデル

  ```shell
  roslaunch nhk2021_simulator 4wd_mecanum_simulation.launch
  ```



上記launchを実行後

```shell
roslaunch pkg_manager_tr activate_TR.launch
```

などのロボット起動launchをたたくとロボットのシミュレーションが可能