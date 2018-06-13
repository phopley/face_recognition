# face_recognition
ROS package for face recognition. Includes the ROS node, a training data generator script and a training script.
## Description
The ROS node, when instructed examines the next frame from the camera for faces. If any faces are detected then an attempt is made to recognise the faces against a known set.
If faces are recognised a message with an array of IDs and names is broadcast. An adjusted image of the image used for the analysis is also broadcast.
If the image contained a detected face then a box is drawn around the face with the predicated name and a predicted level of confidence indication.
Included in the package are two standalone, non ROS, scripts.  
The first takes keyboard input of ID, name and low light boolean. It then uses images from the camera to produce a number of images containing the face of the individual.
These images are used to produce the data sets that will be used to train the face recognition software. This script is run for each individual you wish to recognise.  
The second script iterates through the dataset images and produces a files containing the training data which is loaded by the node and used to train the system for face recognition.
This script should be run whenever the dataset is changed e.g. when a new person is added to the data set.
## Python scripts
- face_recognition_node.py The Python script for the ROS node
- data_set_generator.py The Python script used to produce the data set
- training.py The Python script used to produce the training file from the data set,
## Subscribed topics
- camera/image/raw of type Image from sensor_msgs.msg
- face_recognition_node/start of type Empty from std_msgs.msg
## Published topics
- face_recognition_node/adjusted_image of type Image from sensor_msgs.msg
- face_recognition_node/result of type face_recognition from face_recognition_msgs.msg
## Fixed server parameters
- /face_rec_python/confidence_level, default = 20, minimum threshold value used to decide if the face recognise excepts the result of an identification