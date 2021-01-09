# nhk2021
**NHK Robocon 2021** project  team "**ilias**" (KeioRoboticsAssociation:robot:)



## Git管理上の注意

- masterブランチ　本番環境
- devブランチ　テスト環境



1. masterブランチには基本的には触らないようにしましょう。(devブランチで機能に問題が無い場合は管理者がmasterにマージします)
2. devブランチを用意してあるので作業する人はdev上でブランチを切って作業しましょう。
3. devブランチからは、”機能ごとに”ブランチを切ってください。名前は「featrue_機能名」とします。
4. devにマージする時はプルリクを管理者に送ってください。マージできるか確認したら許可を出すのでマージを行ってください。Slackで報告してくれればさらにgood。
5. 他に管理上気になったことがあったら追記してください。



## 構成

- **Document** - 資料はここに放り込む
- **Serial** - ROSとmbedのシリアル通信
- **TR** - Throwing Robot
- **DR** - Defensive Robot
- **Simulation** - Simulation on Gazebo



## Lisence

The applications are licensed under GPLv3 license.