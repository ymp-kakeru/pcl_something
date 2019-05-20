# pcl_something
pcl関連のもろもろ
## やったこと
1. launchファイルにてvoxel gridとplane segmentは実行できる
2. euclidean clusteringは/devel/lib/pcl_something の実行ファイルからなら
3. super voxel clusteringも同様．ただし、コンパイルエラーなるかも

## やってること
octree使ってDBSCANやる
ただし、octree自体にてまどってしまっている

1. PCDデータの読み取り(以前もやっているのでそのまま流用)
2. octreeの生成
	DBSCAN用に密度がほしいのでOctreePointCloudDensityを使う．ドキュメントが少ないので手探り
3. octreeの各葉ノードから密度情報を抽出 >>>>>>>**(今ここ)**
4. クラスタの生成
	1. 各ボクセルの密度がしきい値以上であればコアorそれ以外(border or outline)
	2. コアボクセルについて，周囲26近傍のボクセルが所属するクラスタに仕分ける．なければ新規のクラスタとする．
	3. それ以外について，周囲26近傍にクラスタがあればそこに仕分ける(border)．なければoutline
5. 各クラスタに色付けしてview

## やらなきゃいけないこと
死ぬこと
笑
笑
笑
笑
笑
