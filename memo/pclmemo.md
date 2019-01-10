# PCL自分用メモ帳
## PCL PointT typeまとめ

1. PointXYZ - float x,y,z  
3Dxyz情報のみを表す．アクセスは
```c++
points[i].data[0]; // [1] [2] x y z の順
points[i].x ;
```

2. PointXYZI - float x,y,z,intensity  
XYZ+intensity(強度)の情報．

3. PointXYZL - float x,y,z; uint32_t label;  
XYZ+label 

4. PointXYZRGBA - float x,y,z; uint32_t rgba;  
XYZ+RGBA 色情報＋透過性を付加

5. PointXYZRGB  
XYZ+RGB

6. InterestPoint - float x,y,z,strength;  
PointXYZIに似ているが，strengthにはキーポイントの強度の尺度が含まれる．

7. Normal - float normal[3], curvature;  
与えられた点でのサーフェス法線と曲率の尺度を持つ．ベクトルへのアクセスは以下
```c++
points[i].data_n[0]; //[1]:y [2]:z [3]:curvature
points[i].normal[0]; 
points[i].normal_x;
```

8. PointNormal - float x,y,z; float normal[3], curvature;  
XYZ+Normal(法線と曲率) アクセスは2つの合わせたやつ

9. PointXYZRGBNormal  
XYZ+RGB+Normal

10. PointXYZINormal  
XYZ+intensity+Normal

11. Narf36 -float x,y,z,roll,pitch.yaw; float descriptor[36]  
XYZ+rpy