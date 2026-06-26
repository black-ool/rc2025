from ultralytics import YOLO
import cv2


model = YOLO('best.onnx')

results = model(
    source=0,
    stream=True
    conf=0.75,  # 这里是置信度选项（可以理解为模型预测图片正确的可能性），低于置信度自动筛除
    max_det=1,    # 限制每张图片所能预测到的最大数量，根据我们的情景，这里应该为1
)

for result in results:
    xywh_tensor = boxes.xywh.cpu().numpy() # 格式: [center_x, center_y, width, height]
    center_point = xywh_tensor[:2]  # 获取中心点

    # 这里是打开可视化窗口，按住“Q”键退出，使用请去掉注释
    # ploted = result.plot()
    # cv2.imshow('frame', ploted)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
