# raspicat_navigation
[![ros-melodic](https://github.com/uhobeike/raspicat_navigation/actions/workflows/raspicat_navigation-ci.yaml/badge.svg)](https://github.com/uhobeike/raspicat_navigation/actions/workflows/raspicat_navigation-ci.yaml)

Raspberry Pi CatでROSの[Navigation Stack](https://wiki.ros.org/navigation)を使ってつくばチャレンジでの複雑な環境等でナビゲーションを行うためのパッケージです。

[Raspberry Pi Cat（Raspberry Pi 4Bのみ）](https://rt-net.jp/products/raspberry-pi-cat/)を用いた[実機でのナビゲーションの動作確認](https://youtu.be/Dgd2tOCEYno)をしています。

## つくばチャレンジでの走行(動画)

[![Image from Gyazo](https://i.gyazo.com/57a22c74ba906adb5e3dbba189a873fa.png)](https://youtu.be/n9tzKY6ua_o)


## 動作環境
### ハードウェア

* [Raspberry Pi Cat](https://rt-net.jp/products/raspberry-pi-cat/)
  * with Raspberry Pi 4B
  * with LiDAR
    * USB
      * UTM-30LX
    * Ether
      * UST-30LX

### ソフトウェア

* Device Driver
  * [rt-net/raspicat_setup_scripts](https://github.com/rt-net/raspicat_setup_scripts)
* ROS
  * [Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
  * [Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* ROS Packages
  * [CIT-Autonomous-Robot-Lab/raspicat](https://github.com/CIT-Autonomous-Robot-Lab/raspicat.git)

## インストール
* 必要なパッケージをまとめてインストール
```
mkdir ~/raspicat_ws/src -p && cd ~/raspicat_ws/
git clone --recursive git@github.com:CIT-Autonomous-Robot-Lab/raspicat.git  ~/raspicat_ws/src/raspicat
rosdep install -r -y --from-paths --ignore-src ./
catkin build
source devel/setup.bash
```

## 使い方
### ウェイポイントファイルの取得(yaml)
* [raspicat_navigation/raspicat_waypoint_navigation/config/waypoint/waypoint.yaml](./raspicat_waypoint_navigation/config/waypoint/waypoint.yaml)に生成されます
```
roslaunch raspicat_waypoint_navigation waypoint_set.launch map_file:=$(rospack find raspit_waypoint_navigation)/config/maps/for_move_base/map_tsukuba.yaml
```

### つくばシミュレータ
#### つくばシミュレータでナビゲーション
```
roslaunch raspicat_waypoint_navigation raspicat_navigation_vsim.launch mcl:=amcl world_name:=tsukuba
rosservice call --wait /way_nav_start
```
#### 手持ちのmapを使ってシミュレータ環境を作成する
```
```

### 実機でナビゲーション
```
```
## ライセンス

このリポジトリはApache License 2.0ライセンスで公開されています。詳細は[LICENSE](./LICENSE)を確認してください。

