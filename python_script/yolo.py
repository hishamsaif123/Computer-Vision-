#!/usr/bin/env python3
import sys
print(sys.executable)
from hisham_msg.msg import BoundingBox,BoundingBoxs 
import rospy 
import cv2 
import torch
from std_msgs.msg import Header 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import numpy as np 
from time import time 
from ultralytics import YOLO 

device = torch.device('cpu')
bridge = CvBridge()
class CameraSubscriber: 

    def __init__(self): 
        #rospy.init_node('camera_subscriber', anonymous=True) 
        #rospy.init_node('camera_publisher', anonymous=True) 
        weight_path = rospy.get_param('~weight_path', './yolov8s.pt') 

        # Initialize YOLOv8 model 
        self.model = YOLO(weight_path,device) # You may need to adjust the initialization based on YOLOv8 requirements 
        

        #self.bounding_boxes = BoundingBoxs() 

        #self.image_sub = rospy.Subscriber('rgb_cam/image_raw', Image, self.capture_frames ) 
        self.imge_pub1 = rospy.Publisher('rgb_cam/image_raw', Image, queue_size = 60) 

        self.yolov8_pub = rospy.Publisher("/Yolov8_Inference", BoundingBoxs, queue_size = 10) 
        self.image_pub = rospy.Publisher("/inference_result", Image, queue_size=10) 
        self.cap = cv2.VideoCapture(0) # 0 is typically the default camera 
        self.camera_frame = 'camera_frame'
        self.bounding_boxes = BoundingBoxs() 
        self.visualize = True
        
        
        if not self.cap.isOpened(): 
            rospy.logerr("Webcam not detected") 
            return 

        rospy.loginfo ("Starting YOLOv8 detection.") 
        

    def run_and_process_detection(self): 
     while not rospy.is_shutdown():
         print("hi")
         ret, frame = self.cap.read() 
         if not ret: 
             rospy.logerr("Failed to grab frame") 
             break 


        # Perform inference 
         results = self.model(frame, show=False, conf=0.3)
         #print(results)
         

        # Process detections 
         self.bounding_boxes.header.frame_id = self.camera_frame
         self.bounding_boxes.header.stamp = rospy.Time.now() 

         for r in results:
             boxes = r.boxes
             for box in boxes:
                 self.bounding_box =  BoundingBox() 
                 b = box.xyxy[0]#.to('cpu').detach().np.copy()
                 c = box.cls
                 self.bounding_box.class_name = self.model.names[int(c)]
                 self.bounding_box.xmin = float(b[0])
                 self.bounding_box.xmax = float(b[1])
                 self.bounding_box.ymin = float(b[2])
                 self.bounding_box.ymax = float(b[3])
                 self.bounding_boxes.bounding_boxes.append(self.bounding_box)

         self.yolov8_pub.publish(self.bounding_boxes)
         self.bounding_boxes.bounding_boxes.clear()

        # Visualize if enabled 
         annotated_frame = r.plot()
         img_msg = bridge.cv2_to_imgmsg(annotated_frame,"bgr8")
         self.image_pub.publish(img_msg) 
         if self.visualize: 
             cv2.imshow('YOLOv8', annotated_frame)  
             cv2.waitKey(1) 


         
         
        
        
        
        
          
    
def main(): 
    rospy.init_node('yolov8_ros', anonymous=True) 
    camera_subscriber = CameraSubscriber() # Corrected from 'yolo_dect = Yolo_Dect()' 
    camera_subscriber.run_and_process_detection( ) 

if __name__== "__main__": 
    main()
