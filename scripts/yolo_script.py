#!/usr/bin/env python3
from time import perf_counter
import rospy
from sensor_msgs.msg import Image
import cv2, numpy as np
from cv_bridge import CvBridge
import vlc
import os

# initialization
class_ids = []
confidences = []
boxes = []
conf_threshold = 0.5
nms_threshold = 0.4
MAX_TIME = 5
sound = None
time_last = 0
time = 0


def play_sound():
    global sound, time_last, time
    print(f"playing sound => {sound} - time {time:.2f}")
    if sound is not None:
        time += perf_counter() - time_last
        time_last = perf_counter()
        if time > MAX_TIME:
            sound = None
            print("reset sound")
            time = 0
        return
    sound = vlc.MediaPlayer(os.path.expanduser("~/Desktop/engel.mp3"))
    print(sound)
    sound.play()
    time_last = perf_counter()


def detect_human(image, best_class, x, y, x_plus_w, y_plus_h, center):
    global timer
    global depth_array
    if classes is not None and str(classes[best_class]) == "person":
        if 480 < center[0] < 1440:
            play_sound()
    distance = round(depth_array[center[1], center[0]], 3)
    if distance == 'nan':
        dist = "Too close or too far way"
    else:
        dist = str(distance)

    label = str(classes[best_class]) + "   Distance: " + dist
    print(label)


def draw_bounding_box(image, best_class, x, y, x_plus_w, y_plus_h, center):

    global depth_array
    distance = round(depth_array[center[1], center[0]], 3)
    if distance == 'nan':
        dist = "Too close or too far way"
    else:
        dist = str(distance)

    try:

        label = str(classes[best_class]) + "   Depth: " + dist

        cv2.circle(image, center, radius=1, color=(255, 0, 0), thickness=1)

        color = COLORS[best_class]

        cv2.rectangle(image, (x, y), (x_plus_w, y_plus_h), color, 2)

        cv2.putText(image, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    except:
        pass


def get_output_layers(net):
    layers = net.getLayerNames()
    out_layers = []
    for i in net.getUnconnectedOutLayers():
        out_layers.append(layers[i - 1])
    return out_layers


def rgba_to_rgb(image):
    row = image.shape[0]
    column = image.shape[1]

    rgb = np.zeros((row, column, 3), dtype='float32')

    r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]

    rgb[:, :, 0] = r
    rgb[:, :, 1] = g
    rgb[:, :, 2] = b

    return np.asarray(rgb, dtype='uint8')


def depth_detection(msg):

    global depth_array

    w = msg.width // 2
    h = msg.height // 2

    index = int(w + (h * msg.width))
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    depth_array = np.array(depth_image, dtype=np.float)


def object_detection(img):
    image = np.frombuffer(img.data,
                          dtype=np.uint8).reshape(img.height, img.width, -1)

    class_ids.clear()
    confidences.clear()
    boxes.clear()

    image = rgba_to_rgb(image)

    Width = img.width
    Height = img.height
    scale = 0.00392

    blob = cv2.dnn.blobFromImage(image,
                                 scale, (224, 224), (0, 0, 0),
                                 True,
                                 crop=False)

    net.setInput(blob)
    outputs = net.forward(get_output_layers(net))

    for out in outputs:
        for det in out:
            class_scores = det[5:]
            best_class = np.argmax(class_scores)
            conf = class_scores[best_class]
            if conf > conf_threshold:
                x_center = int(det[0] * Width)
                y_center = int(det[1] * Height)
                w = int(det[2] * Width)
                h = int(det[3] * Height)
                x = x_center - w / 2
                y = y_center - h / 2
                class_ids.append(best_class)
                confidences.append(float(conf))
                boxes.append([x, y, w, h])

    last_indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold,
                                    nms_threshold)

    for i in last_indices:
        box = boxes[i]
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]

        detect_human(image, class_ids[i], round(x), round(y), round(x + w),
                     round(y + h), (int(x + w / 2), int(y + h / 2)))

        # draw_bounding_box(image,
        #                   class_ids[i], round(x), round(y), round(x + w),
        #                   round(y + h), (int(x + w / 2), int(y + h / 2)))

    #cv2.imshow("object detection", image)

    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    cv2.destroyAllWindows()


if __name__ == "__main__":

    classes = None

    with open("/home/havelsan/Desktop/class_names.txt", 'r') as f:
        classes = [line.strip() for line in f.readlines()]

    COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

    rospy.init_node('yolo', anonymous=True)

    bridge = CvBridge()

    net = cv2.dnn.readNet("/home/havelsan/Desktop/yolov3-tiny.weights",
                          "/home/havelsan/Desktop/yolov3-tiny.cfg")
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    depth_array = None

    rospy.Subscriber('/zedm/zed_node/right/image_rect_color',
                     Image,
                     object_detection,
                     queue_size=1)
    rospy.Subscriber('/zedm/zed_node/depth/depth_registered',
                     Image,
                     depth_detection,
                     queue_size=1)

    rospy.spin()