import os
from ultralytics import YOLO

# Get path
current_dir = os.path.dirname(os.path.abspath(__file__))
data_yaml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(current_dir))), 
                             'hole_detection_real_yolo/data/hole_detection_real.v1i.yolov8/data.yaml')

# Load smaller model
model = YOLO('yolov8n.pt')  # Using nano instead of medium

# Train with reduced batch size
results = model.train(
    data=data_yaml_path,
    epochs=50,           # Reduced epochs
    imgsz=416,          # Reduced image size
    batch=8,            # Smaller batch size
    name='hole_detection'
)