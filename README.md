# 個別要素法による任意の形状をした粒状物質のシミュレータ

*粒状物質150個による崩壊シミュレーション結果*

![giphy](https://user-images.githubusercontent.com/91046559/178481165-cff246a7-e718-486d-8111-1b519c4ee518.gif)  

## バックグラウンド
粒状物質のシミュレーションは粒１つ１つの挙動を計算する「個別要素法」と粒を集合体として扱う「連続体法」の2つに分けられる。

従来の「個別要素法」では粒１つ１つを円として計算している。これは円の接触判定が簡単であり、計算の簡略化のために、粒状物質の形状は円として想定されている。

このシミュレータでは球以外の任意の形状にも対応している。よって、従来の粒状物質シミュレーションとは異なり、円同士の接触では見られない挙動と摩擦も実現し、よりリアリティのある挙動を再現することができる。

## 開発環境
Microsoft Visual Studio Community 2022 (64 ビット) Version 17.0.2

OpenGL

freeglut-3.2.2(剛体の描画)

rapidxml-1.1.3(実験設定ファイルの読み込み)

cereal-1.3.1(粒子のデータをバイナリファイルに保存)


## シミュレーションの流れ
**初期化**

入力ファイルを読み込み粒状物質を配置 or ランダムに粒状物質の配置

**繰り返し処理**
1. 衝突検出（ブロードフェーズ）
    - バウンディングボックスを用いて、衝突が疑われる剛体のペアを検出
2. 衝突検出（ナローフェーズ）
    - WindingNumberを用いて、1で衝突が疑われたペアの中から衝突が確定する粒子を検出する。
    - 点と直線の距離を用いて互いのめり込み量を求める。
3. 衝突力の計算
    - ペナルティ法を用いて、2で求めためり込み量に応じて決まる、衝突によってかかる力を導出する。
4. 剛体の速度・位置の更新
    - 剛体にかかる力から剛体の速度と位置を更新する。
5. 剛体の描画
    - 4で求めた座標から、すべての剛体を描画する
6. n毎にシリアルファイルに保存
    - シミュレーション結果を定期的に保存するために、すべての剛体が持つ位置、速度などの情報をシリアライズする。

## シミュレーション結果
**粒状物質600個によるシミュレーション結果**
![600](https://user-images.githubusercontent.com/91046559/178482827-7c1512f7-1659-4ba0-96c9-5283efb71614.PNG)  　
**粒状物質9600個によるシミュレーション結果**
![9,600](https://user-images.githubusercontent.com/91046559/178482576-caf33ad3-3f0e-4da8-b259-69f177600572.PNG)  


## 工夫した点
  このシミュレータで工夫した点は3点ある。
  
  1点目は衝突検出を2段階に分けることで検出にかかる処理を少なくしている点である。ブロードフェーズで大方の検出を行うことで処理が大きいナローフェーズの処理数を軽減している。
  
  2点目は、WindingNumberを用いることで、円以外の形状をした剛体の衝突判定を可能にしている点である。
  
  3点目は、ペナルティ法を用いた衝突判定を採用している点である。ペナルティ法では、各パラメータをチューニングすることで、シミュレーションにおいて問題になるタイムステップなどの調整が簡単になっている。

## 環境構築手順



## 付録
本シミュレータで使用している数式
-運動方程式と速度・位置の更新式
![equation](https://user-images.githubusercontent.com/91046559/203946215-2f416790-be75-40ca-8a9d-a40b3c09588e.PNG)
![update_pv](https://user-images.githubusercontent.com/91046559/203946287-2a7e86f8-df57-4c54-827a-e8c26b0de0ee.PNG)
- WindingNumber

- ペナルティ法
![F](https://user-images.githubusercontent.com/91046559/203946384-71f76745-9c96-49bb-a064-0c416127008c.PNG)
![Fn](https://user-images.githubusercontent.com/91046559/203946417-a6680eaf-54d5-47ea-a425-979f264d0b94.PNG)

![Ft](https://user-images.githubusercontent.com/91046559/203946446-3f9b3912-8cba-4a17-9e3c-71c6241e7b77.PNG)
![Ft2](https://user-images.githubusercontent.com/91046559/203946451-862aeac7-3acd-404a-ae43-7800ed0a9b67.PNG)


