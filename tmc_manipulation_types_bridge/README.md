開発関係者
----------
* 安田貴夫
* 原田徹平

ToDo
----------
* ソースコード内に記載。（記載方法・・・コメントの初めに「ToDo:」を記述

概要
----------
* 構造体の宣言部、構造体からメッセージ型への変換または逆変換ライブラリを提供する

背景
----------
* 構造体の宣言部、構造体からメッセージ型への変換または逆変換のロジックが  
  様々なノードに点在している状態であったため、一つにまとめることを行った。

前提条件
----------
* 構造体の宣言はtmc\_manipulation\_typesパッケージに宣言されている

機能
----------
* sensor\_msgs/JointStateメッセージからtmc\_manipulation\_types/JointState構造体への変換と、その逆変換。
* tmc\_planning\_msgs/JointPositionメッセージからtmc\_manipulation\_types/Config構造体への変換と、その逆変換。
* tmc\_planning\_msgs/TaskSpaceRegionメッセージからtmc\_manipulation\_types/TaskSpaceRegion構造体への変換と、その逆変換。
* tmc\_planning\_msgs/AttachedObjectメッセージからtmc\_manipulation\_types/AttachedObject構造体への変換。
* tmc\_manipulation\_types/JointTrajectory構造体からtrajectory\_msgs/JointTrajectoryメッセージへの変換。
* tmc\_manipulation\_msgs/CollisionEnvironmentメッセージからtmc\_manipulation\_types/OuterObjectParameterSeqへの変換。
* tmc\_manipulation\_msgs/CollisionEnvironmentメッセージ及びその名称からtmc\_manipulation\_types/CuboidSeqへの変換。
* tmc\_mapping\_msgs/CollisionMapメッセージからtmc\_manipulation\_types/CuboidSeqへの変換。
* tmc\_manipulation\_types/OuterObjectParameters構造体を筆頭に、複数の情報からvisualization\_msgs/Makerメッセージの配列へ変換。
* tmc\_manipulation\_types/Shape構造体を筆頭に、複数の情報からvisualization\_msgs/Markerメッセージへ変換。
