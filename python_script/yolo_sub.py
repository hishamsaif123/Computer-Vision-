#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from hisham_msg.msg import BoundingBox,BoundingBoxs 
from cv_bridge import CvBridge
import cv2

class YoloSubscriber:
    def __init__(self):
        rospy.init_node('yolov8_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/inference_result", Image, self.image_callback)
        self.bboxes_sub = rospy.Subscriber("/Yolov8_Inference", BoundingBoxs , self.bboxes_callback)

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('YOLOv8 Detection Image', cv_image)
        cv2.waitKey(1)

    def bboxes_callback(self, data):
        # Process bounding boxes data
        for bbox in data.bounding_boxes:
            break
            #rospy.loginfo("Detected object: {} with probability: {}".format(bbox.Class, bbox.probability))

if __name__ == '__main__':

    yolo_subscriber = YoloSubscriber()
    rospy.spin()

