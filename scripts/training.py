#!/usr/bin/env python
# This code module is based on that at github.com/thecodacus/Face-Recognition
# and github.com/nazmiarsi95/Face-Recognition
#
# It uses the files produced by DataSetGenerator to produce the training data
# used by the Face Recognition code.

import cv2
import os
import numpy as np

def assure_path_exists(path):
  dir = os.path.dirname(path)
  if not os.path.exists(dir):
    os.makedirs(dir)

# Create Local Binary Patterns Histograms for face recognization
recognizer = cv2.face.LBPHFaceRecognizer_create()

# Using prebuilt frontal face training model, for face detection
detector = cv2.CascadeClassifier("../classifiers/haarcascade_frontalface_default.xml");

# Create method to get the images and label data
def get_images_and_labels(path):

  # Get all file path
  image_paths = [os.path.join(path,f) for f in os.listdir(path)] 
    
  # Initialize empty face sample
  face_samples=[]
    
  # Initialize empty id
  ids = []

  # Loop all the file path
  for image_path in image_paths:

    # The stored image is grayscale so read in in gray scale
    gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Get the image id
    id = int(os.path.split(image_path)[-1].split(".")[1])

    # Get the face from the training images
    # Don't need any scaling as these images already full face
    faces = detector.detectMultiScale(gray);

    # During testing not always detected face on image, which
    # is odd as it should be just an image that was saved
    if (len(faces) == 0):
      print "No face on " + image_path

    else:
      # We know each image is only of one face
      (x, y, w, h) = faces[0]

      # Add the image to face samples
      face_samples.append(gray[y:y+h,x:x+w])

      # Add the ID to IDs
      ids.append(id)

  # Pass the face array and IDs array
  return face_samples,ids

# Get the faces and IDs
faces,ids = get_images_and_labels('dataset')

# Train the model using the faces and IDs
recognizer.train(faces, np.array(ids))

# Save the model into trainer.yml
assure_path_exists('../trainer/')
recognizer.save('../trainer/trainer.yml')

print("Done")
