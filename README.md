# 2018年 ロボットシステム学第2回課題

## 概要
  このパッケージでは、USBカメラを扱うパッケージ(https://github.com/ros-drivers/usb_cam)を利用することで、
  3色(赤、青、黄)のボール(円)を検出することができます。

## 環境
  OS: Ubuntu 16.04
  メインメモリ:  8192 MB
  プロセッサ:      4
  ビデオメモリ:   256 MB
  
  ROS
    バージョン:  kinetic
    利用パッケージ
      usb_cam(https://github.com/ros-drivers/usb_cam)(BSD License)
      
## 利用
  1. usb_cam-test.launch を起動
      ```
      $ roslaunch usb_cam usb_cam-test.launch
      ```
      
  2. ball_search.py を実行
      ```
      $ python ball_search.py
      ```
      
  3. rosrun によりボール検出後の画像を表示
      ```
      $ rosrun image_view image_view image:=/image_result
      ```
      他にも色ごとに設定した閾値でマスクした各色の2値化画像が表示できます。
      ```
      $ rosrun image_view image_view image:=/image_blue &
      $ rosrun image_view image_view image:=/image_red &
      $ rosrun image_view image_view image:=/image_yellow
      ```
      各色のマスクウィンドウを消すとき(同時に表示させている数繰り返す)
      ```
      $ fg
      ```
  4. 終了するとき
      launch, ball_search.py　実行画面
      Ctrl + c
      
  5. rqt_graph を実行することで、ノード同士のつながりを確認できます
      ```
      $ rqt_graph
      ```
      
      
      
      
      
