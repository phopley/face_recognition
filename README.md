# face_recognition

ROS node for face recognition. Included with the node is are a training data generator and a training script.

## Running the Node

Once you have the node built you can test it by launching the test.launch file. This launch file also starts the UbiquityRobotics raspicam_node for the Raspberry Pi camera.

## Node Information
Topics:

* `raspicam_node/image/compressed`:  
  Subscribes `sensor_msgs/CompressedImage` image message to check for recognised faces
  
* `face_recognition_node/image/compressed`:  
  Publishes `sensor_msgs/CompressedImage` the image from the camera with a box around any faces recognised and the name of the identified subject
  
* `face_recognition_node/result`:  
  Publishes `face_recognition/face_recognition_msgs` two arrays containing the ID's and names of any faces recognised

Action:

* `face_recognition_msgs/scan_for_faces`:  
  Server used to control the process of scanning for recognised faces in the current image.

Parameters:

* `/face_rec_python/confidence_level`: Minimum threshold value used to decide if the face recognition is excepted. Default value = 20.

## Helper scripts

Included in the package are two standalone, non ROS, scripts.  
The first (data_set_generator.py) takes keyboard input of ID, name and low light boolean. It then uses images from the camera to produce a number of images containing the face of the individual.
These images are used to produce the data sets that will be used to train the face recognition software. This script is run for each individual you wish to recognise.  
The second script (training.py) iterates through the dataset images and produces a files containing the training data which is loaded by the node and used to train the system for face recognition.
This script should be run whenever the dataset is changed e.g. when a new person is added to the data set.
