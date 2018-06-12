#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import face_recognition_lib
from face_recognition_msgs.msg import face_recognition
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceRecognitionNode:

    def __init__(self):
        self.__image_pub = rospy.Publisher("face_recognition_node/adjusted_image", Image, queue_size=1)

        self.__bridge = CvBridge()
        self.__image_sub = rospy.Subscriber("camera/image/raw",Image, self.callback)

        # The calling node is probably on an action. Therefore we are going to use messages, one to
        # start the process and another to pass on the result.
        self.__command_sub = rospy.Subscriber("face_recognition_node/start",Empty, self.StartCallback)
        self.__response_pub = rospy.Publisher("face_recognition_node/result",face_recognition, queue_size=1)

        # Flag to indicate that we have been requested to use the next image
        self.__scan_next = False
        
        confidence_level = rospy.get_param('/face_rec_python/confidence_level', 20)
        rospy.loginfo("FaceRecognitionNode: Confidence level %s", str(confidence_level))
    
        # Create the face_recognition_lib class instance
        self.__frc = face_recognition_lib.FaceRecognition(roslib.packages.get_pkg_dir("face_recognition", required=True), confidence_level)
  
    # Callback for start command message
    def StartCallback(self, data):
        # Indicate to use the next image for the scan        
        self.__scan_next = True
    
    # Callback for new image received
    def callback(self, data):
        if self.__scan_next == True:
            self.__scan_next = False            
            # The image may show more than one face. Note that the supplied image
            # will be modified if faces are detected. The returned dictionary
            # will contain the unique IDs and Names of any subjects recognised.
            # If no detection/recognition dictionary will be empty            
            image = self.__bridge.imgmsg_to_cv2(data, "bgr8")
      
            detected_dict = self.__frc.scan_for_faces(image) 
    
            try:   
                self.__image_pub.publish(self.__bridge.cv2_to_imgmsg(image, "bgr8"))
            except CvBridgeError as e:
                print(e)
        
            # Now post a message with the list of IDs and names    
            ids = []
            names = []
            for k, v in detected_dict.items():
                ids.append(k)
                names.append(v)
            # Set the message to publish our custom message
            result = face_recognition()
            result.ids_detected = ids
            result.names_detected = names
            self.__response_pub.publish(result)      

def main(args):
    rospy.init_node('face_recognition_node', anonymous=False)
    frn = FaceRecognitionNode()
    rospy.loginfo("Face recognition node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
