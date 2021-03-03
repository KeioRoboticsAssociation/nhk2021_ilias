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

- TRの起動

```shell
roslaunch nhk2021_launcher contorol_TR.launch
```

- DRの起動

```shell
roslaunch nhk2021_launcher contorol_DR.launch
```



## task_selector.pyについて

キーコンフィグについては今後追記

コードを読んで感じてください(適当)



## Trouble Shooting

- bno055が起動できない

  ポートに実行権限を与える

  ```shell
  sudo chmod 666 /dev/ttyACM0
  ```

