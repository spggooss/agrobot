import cv2
import run_model as od
from torchvision import transforms


def run_camera_feed(camera, device):
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    # Check if camera is opened correctly
    if not camera.isOpened():
        print(camera)
        print("Error opening camera")
        exit()

    # Display camera frame op window
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error capturing frame")
            break
        # Display the frame in a window
        cv2.imshow("Camera Feed", frame)
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        image_tensor = transform(frame).to(device)
        image_tensor = image_tensor.unsqueeze(0)


def run_camera_feed_with_model(camera, device, model):
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    # Check if camera is opened correctly
    if not camera.isOpened():
        print(camera)
        print("Error opening camera")
        exit()

    # Display camera frame op window
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error capturing frame")
            break
        # Display the frame in a window
        cv2.imshow("Camera Feed", frame)
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        image_tensor = transform(frame).to(device)
        image_tensor = image_tensor.unsqueeze(0)

        # Run model
        od.detect_objects(model, image_tensor, frame)
