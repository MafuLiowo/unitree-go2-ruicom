from ultralytics import YOLO

model = YOLO("best.pt")
model.export(
    format="onnx",
    imgsz=640,       # 必须固定尺寸
    opset=11,        # 继续保持 11
    simplify=True,   # 导出时自动调用 onnx-simplifier
    dynamic=False    # 务必关闭动态轴
)