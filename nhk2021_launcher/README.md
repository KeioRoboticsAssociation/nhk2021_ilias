# nhk2021_launcher

## これは何

ロボットの起動に必要なparameterやconfig, launchファイルを管理するパッケージ



## config

- ekf_localization

  ekf_localization用のパラメータを記述したyamlが格納されている

- map

  自己位置推定用のmapデータが格納されている

- waypoints

  Pathのwaypointを記述したcsvファイルが格納されている



## launch

以下のコマンドでTRを起動可能

```shell
roslaunch nhk2021_launcher contorol_TR.launch
```

