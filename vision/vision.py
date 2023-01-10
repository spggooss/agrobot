import torch
import cv2
import camera_feed as cf
from torchvision.models import detection

# TODO: tijdelijk een pretrained model gebruikt online

# Define model to use
model = detection.fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

# Move the model to the GPU (is dit nodig?)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

# create camera
camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)


# RUN CAMERA ZONDER MODEL
cf.run_camera_feed(camera, device)

# RUN CAMERA MET MODEL
# cf.run_camera_feed_with_model(camera, device, model)
