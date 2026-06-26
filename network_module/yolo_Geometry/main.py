    from ultralytics import YOLO

    # 1. 加载本地的 .onnx 模型
    model = YOLO('./best.onnx')  #填入模型地址

    # 2. 对图片进行推理
    # 这里的 'path/to/your/image.jpg' 可以是图片路径、文件夹，或者是视频，
    # 使用source=0/1/2/3等可以直接调取摄像头，摄像头处理模板已给予并命名为get_camera.py
    results = model(
        source='path/to/your/image.jpg',
        stream=True
        conf=0.75,  # 这里是置信度选项（可以理解为模型预测图片正确的可能性），低于置信度自动筛除
        max_det=1,    # 限制每张图片所能预测到的最大数量，根据我们的情景，这里应该为1
    )

    # 3. 处理并显示结果
    # 遍历结果列表（通常一张图对应一个 result）
    for result in results:
        boxes = result.boxes
        
        # 1. 获取左上角和右下角坐标 (像素值)
        xyxy_tensor = boxes.xyxy.cpu().numpy()  # 格式: [x1, y1, x2, y2]
        
        # 2. 获取中心点坐标和宽高 (像素值)
        xywh_tensor = boxes.xywh.cpu().numpy()  # 格式: [center_x, center_y, width, height]
        
        # 3. 获取置信度 (Confidence)
        conf_tensor = boxes.conf.cpu().numpy()  # 形状: [N,]
        
        # 4. 获取类别 ID (Class ID)
        cls_tensor = boxes.cls.cpu().numpy()   # 形状: [N,]