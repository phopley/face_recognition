#!/usr/bin/env python
# Face Recognition with OpenCV
# Uses the training data created by using DataSetGenerator and then Training 

import cv2
import os
import yaml

class FaceRecognition:

  def __init__(self, path, confidence):
    # Create Local Binary Patterns Histograms for face recognization
    self.__face_recognizer = cv2.face.LBPHFaceRecognizer_create()

    # Load the trained mode
    self.__face_recognizer.read(path + '/trainer/trainer.yml')

    # Load the names file
    with open(path + '/trainer/names.yml', 'r') as stream:
      self.__names_dict = yaml.load(stream)

    # Detect object in image using Haarcascade Frontal Face
    self.__face_detector = cv2.CascadeClassifier(path + '/classifiers/haarcascade_frontalface_default.xml')

    # Confidence level, the confidence of the system in recognising a face must be greater than
    # this level to be accepted by the system as a recognised face.
    self.__confidence_level = confidence

  # Function to draw rectangle on image according to given (x, y) coordinates 
  # and the given width and height
  def draw_rectangle(self, img, rect, bgr):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), bgr, 2)

  # Function to draw text on give image starting at the passed (x, y) coordinates. 
  def draw_text(self, img, text, x, y, bgr):    
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, bgr, 2)

  # This function detects any faces using OpenCV from the supplied image
  def detect_faces(self, img):
    face_data = []
    
    #convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    

    #let's detect multiscale (some images may be closer to camera than others) images
    #result is a list of faces
    faces_detected = self.__face_detector.detectMultiScale(gray, 1.3, 5);
    
    #if no faces are detected then return None
    if (len(faces_detected) == 0):
      return None, None    
    
    #return only the face part of the image
    for face in faces_detected:
      (x, y, w, h) = face
      face_data.append(gray[y:y+w, x:x+h])

    # faces_detected is a list of rectangles where faces have been detected.
    # face_data is a list of the data for the faces detected
   
    return face_data, faces_detected

  # This class function will be called from outside to scan the supplied img.
  # First it attempts to detect faces in the image and then if any are found
  # it attempts for recognise them against know subjects. It will adjust the
  # supplied image.
  def scan_for_faces(self, img):
    # First do the face detection, returned faces is a list of the faces detected
    # and rects is a list of rectangles of where the faces are in the image
    faces, rects = self.detect_faces(img)

    # Create a dictionary of IDs and Names of those detected in the image
    detected_dict = {}

    # If we detected faces then process each one
    if(faces != None):      
      for index in range(len(faces)):
        # Predict the image using our face recognizer 
        label, confidence = self.__face_recognizer.predict(faces[index])
          
        our_confidence = round(100 - confidence, 2)
        
        # Get name of respective label returned by face recognizer		
        name_text = self.__names_dict[label]       
        name_text_confidence = name_text + " {0:.2f}%".format(our_confidence)
    
        if(our_confidence > self.__confidence_level):
          colour = (0, 255, 0)
        else:
          colour = (0, 0, 255)

        #draw a rectangle around face(s) detected
        self.draw_rectangle(img, rects[index], colour)
        #draw name of predicted person(s) and the confidence value
        self.draw_text(img, name_text_confidence, rects[index,0], rects[index,1]-5, colour)

        if(our_confidence > self.__confidence_level):
	  # Add details to the dictionary to be returned
          detected_dict[label]=name_text

    return detected_dict

