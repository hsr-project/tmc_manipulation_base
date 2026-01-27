開発関係者
================
* 竹下佳佑


目的
================
* 干渉チェックを行う
* あるオブジェクトに対して，近傍のオブジェクトを求める


前提条件
================
* ode-0.12のインストール(0.11.1でも，おそらく動く)

用語説明
================
* グループ/フィルタ<br>
グループとは，干渉チェックの高速化のために行う，オブジェクトのカテゴライズである．<br>
フィルタとは，どのグループと干渉チェックを行うかの設定である．<br>
グループ/フィルタはビットで管理されており，それぞれuint16_t型を用いている．以下に例を示す．<br><br>
オブジェクトA: group = 00000000 00000001, filter = 00000000 11111110<br>
オブジェクトB: group = 00000000 00000001, filter = 00000000 11111110<br>
オブジェクトC: group = 00000000 00000010, filter = 00000000 11111001<br>
オブジェクトD: group = 00000000 00000100, filter = 00000000 11111001<br>
オブジェクトE: group = 00000000 00001000, filter = 00000000 11110111<br><br>
このような場合，オブジェクトA,Bはgroupの最下位bitが1であるので，filterの最下位bitが1となっているオブジェクトC,D,Eと干渉チェックを行う．<br>
オブジェクトCは，オブジェクトA,B,Eと干渉チェックを行うが，オブジェクトDとは干渉チェックを行わない．<br>

* アンカー<br>
複数のオブジェクトの破棄を高速に行うために導入した．<br>
関数 DestroyObject() は，アンカーより後にCreateされたオブジェクトを全て破棄する．<br>
関数 SetAnchor() は，その時点で環境に追加されたオブジェクトの最後尾をアンカーに設定する．<br>
関数 SetAnchor(std::string name) は，オブジェクト名nameのオブジェクトをアンカーに設定する．<br>


自動テスト
================
gtest_collision_detector.cppに記述されている．

* CollisionDetectorTest.CreateObject
    * 正常系
        1. 球を作る
        2. 箱を作る
        3. カプセルを作る
        4. シリンダを作る
        5. メッシュを作る
        6. 箱を10000個作る
    * 異常系
        1. typeが不正
        2. stlファイルが存在しない
        3. stlファイルの中身が空
        4. boxのパラメータが存在しない
        5. sphereのパラメータが負
  
* CollisionDetectorTest.DestroyObject
    * 正常系
        1. オブジェクトを作り破棄
        2. メッシュを作り破棄
    * 異常系 
        1. 何も作っていないオブジェクト破棄
  
* CollisionDetectorTest.UseAnchor
    * 正常系
        1. オブジェクトを作り，最後尾をアンカーにセット
        2. オブジェクトを作り，作ったプリミティブをアンカーにセット
        3. 作ったオブジェクトをアンカーにセットしてから，アンカーの名前を取得
        4. 作ったオブジェクトをアンカーにセットしてから，アンカーを使ってオブジェクトを破棄
        5. 最後のオブジェクトをアンカーにして，アンカーを使ってオブジェクトを破棄
    * 異常系 
        1. アンカーをセットせずに名前を取得
        2. オブジェクトを作らずアンカーセット
        3. アンカーをセットせずに，アンカーを使ってオブジェクトを破棄
  
* CollisionDetectorTest.SetObjectTransform
    * 正常系
        1. オブジェクトを作り，座標をセット
    * 異常系 
        1. Createしていないオブジェクトに座標をセット
  
* CollisionDetectorTest.GetObjectTransform
    * 正常系
        1. オブジェクトを作り，座標をセットしたのち取得
    * 異常系 
        1. Createしていないオブジェクトの座標を取得
  
* CollisionDetectorTest.ChangeObjectPropertyFunctions
    * 正常系 : 機能が正常に働いているかはCheckCollisionSpaceで確認する
    * 異常系 
        1. CreateしていないオブジェクトでSetCollisionGroup()
        2. CreateしていないオブジェクトでSetCollisionFilter()
        3. CreateしていないオブジェクトでEnableObject()
        4. CreateしていないオブジェクトでDisableObject()
  
* CollisionDetectorTest.CheckCollision
    * 正常系 : 正常系はColDetHogeHogeで確認する
    * 異常系 
        1. Createしていないオブジェクトでの干渉チェック
        2. Createしていないオブジェクトでの干渉チェック
  
* CollisionDetectorTest.SphereSphere
* CollisionDetectorTest.SphereBox
* CollisionDetectorTest.SphereCapsule
* CollisionDetectorTest.SphereCylinder
* CollisionDetectorTest.SphereMesh
* CollisionDetectorTest.BoxBox
* CollisionDetectorTest.BoxCapsule
* CollisionDetectorTest.BoxCylinder
* CollisionDetectorTest.BoxMesh
* CollisionDetectorTest.CapsuleCapsule
* CollisionDetectorTest.CapsuleCylinder
* CollisionDetectorTest.CapsuleMesh
* CollisionDetectorTest.CylinderCylinder
* CollisionDetectorTest.CylinderMesh
* CollisionDetectorTest.MeshMesh<br>
形状ごとに干渉チェックが機能しているかを確認する．<br>
2つのオブジェクト間の距離を以下のようにする．<br>
    * 正常系
        1. x軸方向，接触する距離 + kMargin * 3 (接触しない
        2. x軸方向，接触する距離 + kMargin     (接触しない
        3. x軸方向，接触する距離 - kMargin     (接触する
        4. y軸方向，接触する距離 + kMargin * 3 (接触しない
        5. y軸方向，接触する距離 + kMargin     (接触しない
        6. y軸方向，接触する距離 - kMargin     (接触する
        7. z軸方向，接触する距離 + kMargin * 3 (接触しない
        8. z軸方向，接触する距離 + kMargin     (接触しない
        9. z軸方向，接触する距離 - kMargin     (接触する
    * 異常系 : CheckCollisionで確認する
    
* CollisionDetectorTest.CheckCollisionSpace<br>
環境設定：<br>
半径rの球を4つ，中心がxy平面上にあるように配置<br>
全て異なるグループに設定<br>
自分のグループとは干渉しない<br>
球1の中心(0, 0)<br>
球2の中心(3 * r, 0)<br>
球3の中心(0, -1.5 * r)<br>
球4の中心(0, 1.5 * r)<br>
    * 正常系
        1. 何も設定をいじらない，干渉する
        2. 球1を無効化，干渉しない
        3. 球1を無効化したあと有効化，干渉する
        4. 球1のグループとフィルタを球3と同じにする，干渉する
        5. 球1,4のグループとフィルタを球3と同じにする，干渉しない
        6. 球1,2,3,4を破棄する，干渉しない
        7. 球1と球4，球1と球3を干渉チェックから除外，干渉しない
        8. 球1と球4，球1と球3を干渉チェックから除外後，除外リストを破棄，干渉する
        9. 球1,4のグループとフィルタを球3と同じにし，干渉チェックペアに追加，干渉する
    
* CollisionDetectorTest.GetContactPairList
    * 正常系 : 環境設定は上に同じ
        1. 何も設定をいじらない，干渉する(2箇所)
        2. 球1を無効化，干渉しない
        3. 球1を無効化したあと有効化，干渉する(2箇所)
        4. 球1のグループとフィルタを球3と同じにする，干渉する(1箇所)
        5. 球1,4のグループとフィルタを球3と同じにする，干渉しない
        6. 球1,2,3,4を破棄する，干渉しない
        7. 球1と球4，球1と球3を干渉チェックから除外，干渉しない
        8. 球1と球4，球1と球3を干渉チェックから除外後，球1と球3のチェックを有効化，干渉する
        9. 球1と球4，球1と球3を干渉チェックから除外後，除外リストを破棄，干渉する
        10.球1,4のグループとフィルタを球3と同じにし，干渉チェックペアに追加，干渉する

* CollisionDetectorTest.GetClosestObject<br>
環境設定：<br>
半径rの球を中心がxy平面上にあるように配置<br>
球1，フィルタ 100で近傍オブジェクトを検索<br>
球1:中心(0, 0) カテゴリ 001 フィルタ 100<br>
球2:中心(3 * r, 0) カテゴリ 010 フィルタ 100<br>
球3:中心(-4 * r, 0) カテゴリ 100 フィルタ 011<br>
球4:中心(-4 * r, 2 * r) カテゴリ 100 フィルタ 011<br>
球5:中心(-4 * r, 4 * r) カテゴリ 100 フィルタ 011<br>
球6:中心(10 * r, 2 * r) カテゴリ 100 フィルタ 011<br>
    * 正常系
        1. top 3, 拡張 5 * r → 球3
        2. 球3を無効化 top 3, 拡張 5 * r → 球4
        3. top 3, 拡張 0.5 * r → 見つからない
        4. 球3,4,5を無効化 top 3, 拡張 5 * r → 見つからない
        5. top 1, 拡張 5 * r → 何かが見つかる、sphere3だといいな
        6. 球1を無効化してから実行 → 何もしない
    * 異常系 
        1. top 0
        2. 拡張 0
        3. フィルタ 0
        4. 球1を破棄してから実行
    
* CollisionDetectorTest.RayCasting<br>
    * 正常系
        1. 球を対象
        2. 箱を対象
        3. カプセルを対象
        4. シリンダを対象
        5. メッシュを対象
        6. rayの方向にオブジェクトが存在しない
        7. rayの方向にオブジェクトが存在するが遠い
        8. オブジェクトの無効化
    * 異常系
        1. rayの方向が0
        2. rayの長さが0