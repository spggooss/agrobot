import cv2


def detect_objects(model, image_tensor, frame):
    # TODO: model runnen eet framerates van de live camera feed. (like 0.3 FPS)
    # TODO: komt dit omdat ik nu tijdelijk een zwaar, premade model gebruik? Of een trage laptop?
    output = model(image_tensor)

    boxes = output[0]['boxes'].cpu()
    labels = output[0]['labels'].cpu()

    # teken frame boxes van objecten
    # TODO: frame boxes worden nog niet getekend....
    # TODO: idk waarom
    for i in range(len(boxes)):
        box = boxes[i]
        label = labels[i]

        if len(box) != 0:
            # box_width = box[2] - box[0]
            # box_height = box[3] - box[1]
            # cv2.rectangle(frame, (box[0], box[1]), (box[0] + box_width, box[1] + box_height), (0, 0, 255), 2)
            cv2.rectangle(frame, (int(box[2]), int(box[1])), (int(box[0]), int(box[3])), (0, 0, 255), 2)

            print("Start")
            print(frame)
            print(box)
            print("label " + str(label))

            label_str = str(label)
            x, y, w, h = [int(i) for i in box]

            if y - 10 < 0:
                y += 10

            cv2.putText(frame, label_str, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            print("No objects detected in this frame.")
