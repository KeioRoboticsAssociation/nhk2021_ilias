# NHK2021_simulator
## これは何

NHK2021の環境をgazeboで再現するパッケージ。



## ロボットモデル

現在は以下の2つに対応

- 簡易的な差動2輪モデル
- 簡易的な4輪ステアリングモデル (推奨)



## Launch

- フィールド+簡易的な4輪ステアリングモデル

  - TRの場合

    ```
    roslaunch nhk2021_simulator swerve_simulation_TR.launch
    ```

  - DRの場合

    ```
    roslaunch nhk2021_simulator swerve_simulation_DR.launch
    ```

    

- フィールド+簡易的な差動2輪モデル

  ```shell
  roslaunch nhk2021_simulator diff_drive_simulation.launch
  ```



上記launchを実行後

- TR

```shell
roslaunch nhk2021_launcher control_TR.launch
```

- DR

```shell
roslaunch nhk2021_launcher control_DR.launch
```

のロボット起動launchをたたくとロボットのシミュレーションが可能



## Deperecated

4輪メカナムのシミュレーション (必要なし)

こちらのシミュレーターは真面目にジョイント回さずに位置を直接変化させているのでシミュレーターとしての意味はあまりない

cmd_velが正しく出せているかを確認する程度

現在は4輪ステアリングモデルがあるのでこちらを行う必要はない

### 依存パッケージ

- https://github.com/RBinsonB/nexus_4wd_mecanum_simulator

  4輪メカナムのモデル

### Launch

- フィールド+簡易的な4輪メカナムモデル

  ```shell
  roslaunch nhk2021_simulator 4wd_mecanum_simulation_TR.launch # TR
  roslaunch nhk2021_simulator 4wd_mecanum_simulation_DR.launch # DR
  ```
