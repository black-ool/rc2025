使用此.onnx模型推荐安装Ultralytics库（yolo官方库）[pip install ultralytics]
使用方法见 -> main.py

如果不使用Ultralytics库，则可以使用 onnx, onnxruntime 库，但此库不会自动预处理，需要使用者在传入图片时人工将图片缩放为640*640。

模型表现最佳（F1 score）的置信度参数为 conf=0.808  ，给予代码中以设置为0.75以偏重召回率，防止漏报。

模型输出的索引序列以及其标签参照：
  0: sphere # 球体
  1: tetrahedron # 三棱锥
  2: cube # 正方体
  3: cylinder # 圆柱体
