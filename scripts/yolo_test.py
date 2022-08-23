#!/usr/bin/env python3
from pickletools import bytes8
import rospy
from sensor_msgs.msg import Image
import cv2, numpy as np
import PIL.Image
import os
from codecs import decode
import struct
from cv_bridge import CvBridge

bridge = CvBridge()
depth_array = None


def get_output_layers(net):

    layer_names = net.getLayerNames()
    #print(net.getUnconnectedOutLayers()[0])
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers


# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h,
                      center):
    global depth_array

    try:
        label = str(classes[class_id]) + "   Depth: " + str(
            depth_array[center[1], center[0]])
        cv2.circle(img, center, radius=1, color=(255, 0, 0), thickness=1)
        #print(label)
        color = COLORS[class_id]

        cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)

        cv2.putText(img, label + "test", (x - 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    except:
        pass

    # run inference through the network


def rgba2rgb(rgba, background=(255, 255, 255)):
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros((row, col, 3), dtype='float32')
    r, g, b, a = rgba[:, :, 0], rgba[:, :, 1], rgba[:, :, 2], rgba[:, :, 3]

    a = np.asarray(a, dtype='float32') / 255.0

    R, G, B = background

    rgb[:, :, 0] = r * a + (1.0 - a) * R
    rgb[:, :, 1] = g * a + (1.0 - a) * G
    rgb[:, :, 2] = b * a + (1.0 - a) * B

    return np.asarray(rgb, dtype='uint8')


counter = 0


def depth_detecting(msg):
    global depth_array
    w = msg.width // 2
    h = msg.height // 2
    index = int(w + (h * msg.width))
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
    depth_array = np.array(depth_image, dtype=np.float)
    #print("Depth: ", np.unique(depth_array))


def line_detecting(img_raw):
    global counter
    counter += 1
    if True:
        counter = 30
        image = np.frombuffer(img_raw.data,
                              dtype=np.uint8).reshape(img_raw.height,
                                                      img_raw.width, -1)
        image = rgba2rgb(image)
        Width = img_raw.width
        Height = img_raw.height
        scale = 0.00392

        # create input blob
        blob = cv2.dnn.blobFromImage(image,
                                     scale, (416, 416), (0, 0, 0),
                                     True,
                                     crop=False)

        # set input blob for the network
        net.setInput(blob)

        # function to get the output layer names
        # in the architecture

        # and gather predictions from output layers
        outs = net.forward(get_output_layers(net))

        # initialization
        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        # for each detetion from each output layer
        # get the confidence, class id, bounding box params
        # and ignore weak detections (confidence < 0.5)
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * Width)
                    center_y = int(detection[1] * Height)
                    w = int(detection[2] * Width)
                    h = int(detection[3] * Height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        # apply non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold,
                                   nms_threshold)

        # go through the detections remaining
        # after nms and draw bounding box
        for i in indices:
            i = i
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]

            draw_bounding_box(image, class_ids[i], confidences[i], round(x),
                              round(y), round(x + w), round(y + h),
                              (int(x + w / 2), int(y + h / 2)))

        # display output image
        cv2.imshow("object detection", image)

        # wait until any key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


if __name__ == "__main__":
    #cmd1 = 'export PYTHONPATH=~/opencv/build/lib/python3/'
    #cmd2='. /opt/ros/melodic/.setup.bash'
    #os.system(cmd1)
    #os.system(cmd2)
    rospy.init_node('hareket', anonymous=True)
    #pub = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 1)
    # read class names from text file
    classes = None
    with open("/home/havelsan/Desktop/class_names.txt", 'r') as f:
        classes = [line.strip() for line in f.readlines()]

    # generate different colors for different classes
    COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

    # read pre-trained model and config file
    net = cv2.dnn.readNet("/home/havelsan/Desktop/yolov3-tiny.weights",
                          "/home/havelsan/Desktop/yolov3-tiny.cfg")
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    rospy.Subscriber('/zedm/zed_node/right/image_rect_color',
                     Image,
                     line_detecting,
                     queue_size=1)
    rospy.Subscriber('/zedm/zed_node/depth/depth_registered',
                     Image,
                     depth_detecting,
                     queue_size=1)
    #msg = AckermannDriveStamped()
    #msg.drive.speed = 0
    #msg.drive.acceleration = 1;
    #msg.drive.jerk = 1;
    #msg.drive.steering_angle = 0
    #msg.drive.steering_angle_velocity = 1
    """
  while True:
      if not flag:
          exit()
  """

    rospy.spin()
