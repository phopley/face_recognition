#!/usr/bin/env python
# This code module is based on that at github.com/thecodacus/Face-Recognition
# and github.com/nazmiarsi95/Face-Recognition
#
# It uses the Raspberry Pi Camera to capture facial images and stores them
# ready for the face recognition training phase. The script should be run
# for each subject that you wish to recognise.
# Ensure when prompted you give the next ID value (start at 1) and increase
# for each subject recorded

import cv2
import os
import io
import numpy
import yaml
import picamera

# Detect object in video stream using Haarcascade Frontal Face
face_detector = cv2.CascadeClassifier('../classifiers/haarcascade_frontalface_default.xml')

def assure_path_exists(path):
  dir = os.path.dirname(path)
  if not os.path.exists(dir):
    os.makedirs(dir)

assure_path_exists("dataset/")
assure_path_exists("../trainer/")

with picamera.PiCamera() as camera:
  camera.resolution = (1280, 960)

  looping = True
  count = 0
  end = 99
  names_dict = {}
  name_file = '../trainer/names.yml'

  # Open the file of IDs and names to append the new one to
  if os.path.exists(name_file):
    with open(name_file, 'r') as stream:
      names_dict = yaml.load(stream)

  cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

  face_id = raw_input("What is this persons ID number? ")
  name = raw_input("What is this persons name? ")
  low_light = raw_input("Low light Y/N?" )

  if low_light == 'Y' or low_light == 'y':
    count = 100
    end = 199

  # If not already in the dictionary add details
  if not face_id in names_dict:
    names_dict[int(face_id)]=name

  with open(name_file, 'w') as outfile:
    yaml.dump(names_dict, outfile, default_flow_style=False)

  while(looping):
    # Create a memory stream so image doesn't need to be saved to a file
    stream = io.BytesIO()

    camera.capture(stream, format='jpeg')

    #Convert picture to numpy array
    buff = numpy.fromstring(stream.getvalue(), dtype=numpy.uint8)

    # Now create an OpenCV image
    image_frame = cv2.imdecode(buff, 1)

    # Convert frame to grayscale
    gray = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)

    # Detect frames of different sizes, list of faces rectangles
    faces = face_detector.detectMultiScale(gray, 1.3, 5)

    # Although faces could contain more than one face we only expect one
    # person to be in the data set image otherwise it would confuse
    # the whole thing
    if (len(faces) != 0):
      # Expecting one face only on the data set image
      (x, y, w, h) = faces[0]

      # Crop the image frame into rectangle
      cv2.rectangle(image_frame, (x,y), (x+w,y+h), (255,0,0), 4)

      # Increment sample face image
      count += 1

      # Save the captured image into the datasets folder
      cv2.imwrite("dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])

      # Display the video frame, with bounded rectangle on the person's face
      cv2.imshow('frame', image_frame)

    # To stop taking video, press 'q' for at least 100ms
    if cv2.waitKey(100) & 0xFF == ord('q'):
      looping = False

    # If image taken reach 100, stop taking video
    elif count>end:
      looping = False

  # Close all started windows
  cv2.destroyAllWindows()

  print("Data prepared")

