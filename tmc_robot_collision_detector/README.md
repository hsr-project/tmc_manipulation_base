tmc_robot_collision_detector {#mainpage}
===================================================

開発関係者
================
* 竹下佳佑

目的
================
* ロボットの内部/外部干渉チェックを行う

前提条件
================
* ロボットの運動学モデルを提供するクラスIRobotKinematicsModelの実装がある
* 干渉チェックを行うクラスICollisionDetectorの実装がある

用語説明
================
* グループ/フィルタ<br>
干渉チェックのためのオブジェクトのグループ化と，
そのグループ間で干渉チェックを行うかのフィルタリングである．
詳しい仕組みはcollision_detectorのREADMEに記述されている．

* 親オブジェクト/子オブジェクト<br>
複数のプリミティブから構成されるオブジェクトを，干渉チェック空間に追加するための呼称である．
例えば，空間に"机"を追加する場合，"机"が親オブジェクトであり，
"机の脚"，"天板"が子オブジェクトである．
空間にオブジェクトを追加する際には親オブジェクトの名前から子オブジェクトの名前が決定され，
親オブジェクトの名前が"table"であれば，子オブジェクトは"table#0","table#1",…となる．

干渉チェックの有効化/無効化
================
あるオブジェクトとオブジェクトの間で干渉チェックが行われるかどうかの設定は3種類ある．

* グループ/フィルタ<br>
説明は上記の通りである．グループ/フィルタを設定/取得する関数を以下に示す．
DisableCollisionCheckGroupToGroup(),
EnableCollisionCheckGroupToGroup()を除いて，
グループ/フィルタを設定する関数は現在のグループ/フィルタ設定を
無視して設定を書き換えるため注意が必要である．
    * DisableCollisionCheckGroupToGroup()
    * EnableCollisionCheckGroupToGroup()
    * SetObjectGroup()
    * SetObjectDefaultGroup()
    * GetObjectGroup()
    * GetObjectDefaultGroup()
    * SetObjectFilter()
    * SetObjectDefaultFilter()
    * GetObjectFilter()
    * GetObjectDefaultFilter()
    * HoldObject()
    * ReleaseObject()
    * ReleaseAllObject()

* オブジェクトの有効化/無効化<br>
通常，作られたオブジェクトに対して干渉チェックが行われるが，これを無効化することができる．
1度無効化すると，有効化するまで干渉チェックから省かれる．
設定は以下の関数で行うことができる．
    * EnableCollisionObject()
    * DisableCollisionObject()

* 干渉チェック除外/追加ペアリスト<br>
オブジェクトとオブジェクト/グループ間でグループ/フィルタ，
オブジェクトの有効化/無効化とは別に干渉チェックの除外/追加が出来る．
1度リストに追加されると，それを無効化するように除外/追加を行うか，
全てのオブジェクトの破棄されるまで有効となる．
設定は以下の関数で行うことができる．
    * DisableCollisionCheckObjectToObject()
    * EnableCollisionCheckObjectToObject()
    * DisableCollisionCheckObjectToGroup()
    * EnableCollisionCheckObjectToGroup()


上記の例のように`<mesh>`タグに package を追加した．rosのパッケージ名を書いておけば展開される．

URDFで干渉チェックを行う際の注意点
================================

`model_file_type`に`tmc_robot_collision_detector::kUrdf`を指定し，
`robot_collision_config`のshapeの指定は

(Link名)+/collision/+(shapeの通し番号0~)

とすること．

例:

elbow_roll/collision/0

これは，URDFではcollisionのタグに名前をつけないのが一般的であるため，
このライブラリでは/collision/+(shapeの通し番号0~)をつける．

干渉チェック設定ファイルの書き方の例
================
* グループ設定<br>
`<group name= "HEAD">`<br>
`  <object name="CARM/HEAD/SHAPE_NECK"/>`<br>
`  <object name="CARM/HEAD/SHAPE_HEAD"/>`<br>
`  <object name="CARM/HEAD/SHAPE_URG"/>`<br>
`  <object name="CARM/HEAD/SHAPE_URG_PEDESTAL"/>`<br>
`  <object name="CARM/HEAD/SHAPE_KINECT"/>`<br>
`</group>`<br>
ロボット設定ファイルの部位名をobjectのnameに書く．
必ず他の設定より先にグループ設定を書くこと．

* ロボット部位のグループ名の設定<br>
`<robot-parts-group>`<br>
`  <group name="BODY"/>`<br>
`  <group name="LINEAR"/>`<br>
`  <group name="HEAD"/>`<br>
`  <group name="HAND"/>`<br>
`  <group name="ARM1"/>`<br>
`  <group name="ARM2"/>`<br>
`  …`<br>
`</robot-parts-group>`<br>
上のグループ設定で設定したグループ名を書く．
存在しないグループ名が書かれているとコンストラクタが例外を投げる．

* 干渉チェックを行わないグループの設定<br>
`<non-contact>`<br>
`  <pair group1="BODY" group2="HEAD"/>`<br>
`  <pair group1="BODY" group2="ARM1"/>`<br>
`</non-contact>`<br>
pairのgroup1，group2にそれぞれ上記で設定したグループ名を書く．
存在しないグループ名が書かれているとコンストラクタが例外を投げる．

* ロボット内部で近傍オブジェクトを探す際に無視するグループの設定<br>
`<non-check-inner-distance>`<br>
`  <pair group1="ARM5" group2="ARM7" />`<br>
`  <pair group1="ARM1" group2="ARM3" />`<br>
`  <pair group1="ARM3" group2="LINEAR" />`<br>
`</non-check-inner-distance>`<br>
ロボット内部で近傍オブジェクトを探す際，上記の干渉チェックを行わないグループに追加して，
無視するグループを設定できる．
pairのgroup1，group2に設定したいグループ名を書く．
存在しないグループ名が書かれているとコンストラクタが例外を投げる．

Cuboidの扱い
================
Cuboidを扱う関数は以下の3つ
* void CreateCuboids
* void RefleshOverlappedCuboids
* void DestroyCuboids

サンプル
================
tmc_bmc_robot_collision_detectorに存在する．

* sample`_`check`_`closest`_`object<br>
ロボット，ロボット前方の壁，ロボット左前方のボックスから構成される空間で，
ロボットの手先から最も近いオブジェクト及びロボット部位を導出し，rvizで表示する．
ロボット関節情報はjoint`_`stateから取得する．<br>
実行例: rosrun tmc`_`robot`_`collision`_`detector sample`_`check`_`closest`_`object

* sample`_`overall<br>
RobotCollisionDetectorが提供する各種関数を用いている．<br>
実行例: ./bin/sample`_`overall


自動テスト
================

* CollisionDetectorConfigTest.Constructor
    * 正しい設定ファイルを読み込む
    * 存在しない設定ファイルを指定
    * 文法ミスのある設定ファイルを指定
    * グループ数が上限を越える設定ファイルを読み込む
    * 存在しないグループを干渉チェック除外ペアとした設定ファイルを読み込む
    * 存在しないグループを干渉チェック除外ペアとした設定ファイルを読み込む

* CollisionDetectorConfigTest.GetBitByObject
    * オブジェクト名を指定してグループbitを取得
    * オブジェクト名を指定してフィルタbitを取得
    * オブジェクト名を指定してInner bitを取得
    * ロボット部位ではないオブジェクト名を指定してInner bitを取得

* CollisionDetectorConfigTest.GetBitByGroup
    * グループ名を指定してグループbitを取得
    * グループ名を指定してフィルタbitを取得
    * 存在しないグループ名を指定してグループbitを取得
    * 存在しないグループ名を指定してフィルタbitを取得

* CollisionDetectorConfigTest.SetConfig
    * 設定して変更が反映される
    * 存在しないグループ名を指定して設定

* CollisionDetectorConfigTest.GetName
    * オブジェクト名からグループ名を取得
    * 存在しないオブジェクトのグループを取得する
    * グループに所属するオブジェクトのリストを取得
    * ロボット部位のグループ名リストを取得
    * 干渉チェックから除外するオブジェクトペアのリストを取得
    * 存在しないグループ名を指定してグループに所属するオブジェクトのリストを取得

* RobotCollisionDetectorTest.Constructor
    * 正規のファイル(ODE)を読み込む
    * ロボットモデルファイルが存在しない
    * ロボットモデルファイルの文法が正しくない
    * 存在しない物理エンジンを指定

* RobotCollisionDetectorTest.CreateOuterObject
    * 正常系
    * 子物体のshapeとposeの数が一致しない
    * 名前がない
    * 既にある名前を使おうとしている

* RobotCollisionDetectorTest.CreateCuboids
    * 干渉チェックを有効にする
    * 干渉チェックを無効にする
    * グループを指定してCuboidを作成する
    * 同じCuboidを作成する
    * 名前が空のCuboidを作成する
    * 存在しないグループを指定してCuboidを作成する

* RobotCollisionDetectorTest.DestroyOuterObject
    * 存在する外部オブジェクトを破棄
    * ロボット部位を破棄(破棄しない)
    * 掴んでいるオブジェクトを破棄(破棄しない)
    * 全ての外部オブジェクトを破棄
    * 外部オブジェクトを作成せず，全ての外部オブジェクトを破棄
    * Cuboidを破棄
    * 存在しない外部オブジェクトを破棄
    * 一度破棄したオブジェクトを再び破棄
    * 子オブジェクトを破棄

* RobotCollisionDetectorTest.DestroyCuboids
    * Cuboidを破棄

* RobotCollisionDetectorTest.GetObjectParameter
    * 作成したオブジェクトのパラメータを取得
    * ロボット部位のパラメータを取得
    * 子オブジェクトのパラメータを取得
    * Cuboidのパラメータを取得
    * 存在しないオブジェクトのパラメータを取得

* RobotCollisionDetectorTest.GetAllOuterObjectParameter
    * 外部オブジェクトがない場合
    * 外部オブジェクトが2個ある場合
    * Cuboidがある場合

* RobotCollisionDetectorTest.EnableDisableObject
    * 外部オブジェクトに対するEnableObjectの動作チェック
    * 外部オブジェクトに対するDisableObjectの動作チェック
    * ロボット部位に対するEnableObjectの動作チェック
    * ロボット部位に対するDisableObjectの動作チェック
    * 子オブジェクトに対するEnableObjectの動作チェック
    * 子オブジェクトに対するDisableObjectの動作チェック
    * Cuboidに対するEnableObjectの動作チェック
    * Cuboidに対するDisableObjectの動作チェック
    * 存在しないオブジェクトに対するEnableObjectの動作チェック
    * 存在しないオブジェクトに対するDisableObjectの動作チェック

* RobotCollisionDetectorTest.OperateGroupProperty
    * 外部オブジェクトに対するGetObjectGroupの動作チェック
    * 外部オブジェクトに対するSetObjectGroupの動作チェック
    * 外部オブジェクトに対するSetObjectDefaultGroupの動作チェック
    * ロボット部位に対するGetObjectGroupの動作チェック
    * ロボット部位に対するSetObjectGroupの動作チェック
    * ロボット部位に対するSetObjectDefaultGroupの動作チェック
    * 子オブジェクトに対するGetObjectGroupの動作チェック
    * 子オブジェクトに対するGetObjectDefaultGroupの動作チェック
    * 子オブジェクトに対するSetObjectGroupの動作チェック
    * 子オブジェクトに対するSetObjectDefaultGroupの動作チェック
    * Cuboidに対するGetObjectGroupの動作チェック
    * Cuboidに対するSetObjectGroupの動作チェック
    * Cuboidに対するSetObjectDefaultGroupの動作チェック
    * 存在しないオブジェクトに対するGetObjectGroupの動作チェック
    * 存在しないオブジェクトに対するSetObjectGroupの動作チェック
    * 存在しないオブジェクトに対するSetObjectDefaultGroupの動作チェック

* RobotCollisionDetectorTest.OperateFilterProperty
    * 外部オブジェクトに対するGetObjectFilterの動作チェック
    * 外部オブジェクトに対するSetObjectFilterの動作チェック
    * 外部オブジェクトに対するSetObjectDefaultFilterの動作チェック
    * ロボット部位に対するGetObjectFilterの動作チェック
    * ロボット部位に対するSetObjectFilterの動作チェック
    * ロボット部位に対するSetObjectDefaultFilterの動作チェック
    * 子オブジェクトに対するGetObjectFilterの動作チェック
    * 子オブジェクトに対するGetObjectDefaultFilterの動作チェック
    * 子オブジェクトに対するSetObjectFilterの動作チェック
    * 子オブジェクトに対するSetObjectDefaultFilterの動作チェック
    * Cuboidに対するGetObjectFilterの動作チェック
    * Cuboidに対するSetObjectFilterの動作チェック
    * Cuboidに対するSetObjectDefaultFilterの動作チェック
    * 存在しないオブジェクトに対するGetObjectFilterの動作チェック
    * 存在しないオブジェクトに対するSetObjectFilterの動作チェック
    * 存在しないオブジェクトに対するSetObjectDefaultFilterの動作チェック

* RobotCollisionDetectorTest.DisableCollisionPairProperty
    * 干渉チェックから外す(ロボット部位/親オブジェクト)
    * 干渉チェックから外す(ロボット部位/子オブジェクト)
    * 干渉チェックから外す(ロボット部位/外部オブジェクト)
    * 干渉チェックから外す(部位グループ/親オブジェクト)
    * 干渉チェックから外す(部位グループ/子オブジェクト)
    * 干渉チェックから外す(部位グループ/外部オブジェクト)
    * グループの設定変更が反映されているか
    * 存在しないオブジェクト名を指定(object object)
    * 存在しないオブジェクト名を指定(object group)
    * 存在しないグループ名を指定(object group)
    * 存在しないグループ名を指定(group group)

* RobotCollisionDetectorTest.EnableCollisionPairProperty
    * 干渉チェックに追加(ロボット部位/親オブジェクト)
    * 干渉チェックに追加(ロボット部位/子オブジェクト)
    * 干渉チェックに追加(ロボット部位/外部オブジェクト)
    * 干渉チェックに追加(部位グループ/親オブジェクト)
    * 干渉チェックに追加(部位グループ/子オブジェクト)
    * 干渉チェックに追加(部位グループ/外部オブジェクト)
    * 存在しないオブジェクト名を指定(object object)
    * 存在しないオブジェクト名を指定(object group)
    * 存在しないグループ名を指定(object group)
    * 存在しないグループ名を指定(group group)

* RobotCollisionDetectorTest.GetObjectTransform
    * ロボット部位(shape)の姿勢を取得
    * ロボット部位(joint)の姿勢を取得
    * 外部オブジェクトの姿勢を取得
    * 把持オブジェクトの姿勢を取得
    * Cuboidの姿勢を取得
    * 存在しないオブジェクトの姿勢を取得

* RobotCollisionDetectorTest.SetObjectTransform
    * 外部オブジェクトの姿勢を設定
    * ロボット部位(shape)の姿勢を設定(設定できない)
    * 把持オブジェクトの姿勢を設定(設定できない)
    * Cuboidの姿勢を設定
    * 存在しないオブジェクトの姿勢を設定

* RobotCollisionDetectorTest.HoldObject
    * 子オブジェクトが１つのオブジェクトを把持
    * 子オブジェクトが複数のオブジェクトを把持
    * 既に把持しているオブジェクトを把持(把持できない)
    * Cuboidを把持
    * ロボット部位を把持(把持できない)
    * 存在しないオブジェクトを把持
    * 存在しないフレームで把持
    * 存在しないグループで把持

* RobotCollisionDetectorTest.ReleaseObject
    * 子オブジェクトが１つのオブジェクトを解放
    * 子オブジェクトが複数のオブジェクトを解放
    * 全ての把持オブジェクトを解放
    * Cuboidを手放す
    * 存在しないオブジェクトを解放
    * 把持していないオブジェクトを解放

* RobotCollisionDetectorTest.CollisionCheck
    * 干渉する
    * 干渉チェック無効化→干渉しない
    * 無効化後有効化→干渉する
    * 把持→干渉しない
    * 把持後解放→干渉する
    * フィルタ設定をBODYと干渉しないように変える→干渉しない
    * グループを元に戻す→干渉する
    * 干渉物体ペア名取得
    * 干渉物体ペア名リスト取得
    * デフォルトの干渉チェック除外リストが有効か確認

* RobotCollisionDetectorTest.CheckClosestObject
    * 全ロボット部位で探索，把持なし
    * 全ロボット部位で探索，把持あり
    * ロボット部位で検索
    * 把持オブジェクトで検索
    * 1つのオブジェクトから構成される外部オブジェクトで検索
    * 複数のオブジェクトから構成される外部オブジェクトで検索
    * 存在しないオブジェクトで検索

* RobotCollisionDetectorTest.GetAABB
    * ロボットのAABBを取得
    * 親オブジェクトのAABBを取得
    * 子オブジェクトのAABBを取得
    * 存在しないオブジェクトのAABBを取得

* RobotCollisionDetectorTest.RefleshOverlappedCuboids
    * 正規の動作(aabb, group)
    * 正規の動作(2dmap, group)
    * 正規の動作(aabb, robot)
    * 正規の動作(2dmap, robot)

* RobotCollisionDetectorTest.EnableDisableCuboids
    * 有効にする
    * 無効にする
    * CUBOIDは０だが，存在するグループを指定して有効化
    * CUBOIDは０だが，存在するグループを指定して無効化
    * 存在しないグループを有効化
    * 存在しないグループを有効化

* RobotCollisionDetectorTest.GetNameList
    * 空間内の全てのオブジェクト名のリストを取得
    * 外部オブジェクトグループに含まれるオブジェクト名のリストを取得
    * 把持オブジェクトがあるグループに含まれるオブジェクト名のリストを取得
    * ロボット部位グループに含まれるオブジェクト名のリストを取得
    * CUBOIDグループに含まれるオブジェクト名のリストを取得
    * CUBOIDが追加されたグループに含まれるオブジェクト名のリストを取得
    * オブジェクトが空のグループを取得する
    * グループ名のリストを取得
    * 把持物体名のリストを取得
    * NormalCase2で存在しないグループ名を指定
