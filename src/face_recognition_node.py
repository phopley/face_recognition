#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import face_recognition_lib
import actionlib
from face_recognition_msgs.msg import scan_for_facesAction, scan_for_facesGoal, scan_for_facesResult
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class FaceRecognitionNode:

    def __init__(self):
        self.__bridge = CvBridge()
        self.__image_pub  = rospy.Publisher('face_recognition_node/image/compressed', CompressedImage, queue_size=1)
        self.__image_sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, self.callback)
       
        confidence_level = rospy.get_param('/face_rec_python/confidence_level', 20)
        rospy.loginfo("FaceRecognitionNode: Confidence level %s", str(confidence_level))
    
        # Create the face_recognition_lib class instance
        self.__frc = face_recognition_lib.FaceRecognition(roslib.packages.get_pkg_dir('face_recognition', required=True), confidence_level)
 
        # Create the Action server
        self.__as = actionlib.SimpleActionServer('face_recognition', scan_for_facesAction, self.do_action, False)
        self.__as.start()

    def do_action(self, goal):
        # Scan the current image for faces recognised
        # The image may show more than one face.
        # The returned dictionary will contain the unique IDs
        # and Names of any subjects recognised.
        # If no detection/recognition dictionary will be empty
        image = self.__bridge.compressed_imgmsg_to_cv2(self.__current_image)
      
        # In the next call image will be altered if faces are recognised
        detected_dict = self.__frc.scan_for_faces(image) 
    
        try:   
            self.__image_pub.publish(self.__bridge.cv2_to_compressed_imgmsg(image))                
        except CvBridgeError as e:
            print(e)
        
        # Now post a message with the list of IDs and names    
        ids = []
        names = []
        for k, v in detected_dict.items():
            ids.append(k)
            names.append(v)
        # Set result for the action
        result = scan_for_facesResult()
        result.ids_detected = ids
        result.names_detected = names
        self.__as.set_succeeded(result)        
    
    # Callback for new image received
    def callback(self, data):
        # Each time we receive an image we store it ready in case then asked to scan it
        self.__current_image = data
                  

def main(args):
    rospy.init_node('face_recognition_node', anonymous=False)
    frn = FaceRecognitionNode()
    rospy.loginfo('Face recognition node started')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main(sys.argv)
