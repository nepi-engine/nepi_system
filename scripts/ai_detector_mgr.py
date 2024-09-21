#!/usr/bin/env python

import sys
import os
import os.path
import errno
import glob
import subprocess
import yaml
import time
import rospy
import numpy as np
import cv2

from nepi_edge_sdk_base import nepi_ros 
from nepi_edge_sdk_base import nepi_pc

from std_msgs.msg import Empty, Float32
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from nepi_ros_interfaces.srv import ImageClassifierListQuery, ImageClassifierListQueryResponse
from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse, SystemStorageFolderQuery
from nepi_ros_interfaces.msg import BoundingBoxes, ObjectCount,ClassifierSelection, StringArray, TargetLocalization


from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF



class AIDetectorManager:

    # AI Detection Setttings
    NODE_NAME = "ai_detector_mgr"
    DARKNET_CFG_PATH = '/mnt/nepi_storage/ai_models/darknet_ros/'
    MIN_THRESHOLD = 0.001
    MAX_THRESHOLD = 1.0
    FIXED_LOADING_START_UP_TIME_S = 5.0 # Total guess
    ESTIMATED_WEIGHT_LOAD_RATE_BYTES_PER_SECOND = 16000000.0 # Very roughly empirical based on YOLOv3


    darknet_cfg_files = []
    classifier_dict = dict()

    current_classifier = "None"
    current_classifier_classes = []
    current_model_type = "None"
    current_img_topic = "None"
    has_depth_map = False
    current_depth_map_topic = "None"
    has_pointcloud = False
    current_pointcloud_topic = "None"
    current_threshold = 0.3
    classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED
    classifier_load_start_time = None

    darknet_ros_process = None
    darknet_update_sub= None
    darknet_found_object_sub = None
    darknet_bounding_boxes_sub = None


    def __init__(self):
        # Find Darknet Classifier Models
        [darknet_classifiers,darknet_model_sizes,darknet_model_classes] = self.darknetFindClassifiers()

        for i in range(len(darknet_classifiers)):
            entry = {'type': 'Darknet', 'size': darknet_model_sizes[i], 'classes': str(darknet_model_classes[i])}
            self.classifier_dict[darknet_classifiers[i]] = entry
        
        if (len(self.classifier_dict.keys()) < 1):
            rospy.logwarn("No classiers identified for this system at %s", darknet_cfg_path_config_folder)


        # Setup Node Services
        rospy.Service('~img_classifier_list_query', ImageClassifierListQuery, self.provideClassifierList)
        rospy.Service('~img_classifier_status_query', ImageClassifierStatusQuery, self.provideClassifierStatus)
        # Setup Constant Node Subscribers
        rospy.Subscriber('~start_classifier', ClassifierSelection, self.startClassifierCb)
        rospy.Subscriber('~stop_classifier', Empty, self.stopClassifierCb)
        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb) 

        # Setup config IF system
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        # Load default params
        #namespace = rospy.get_namespace()
        self.updateFromParamServer()


    def startClassifierCb(self, classifier_selection_msg):
        classifier=classifier_selection_msg.classifier
        input_img=classifier_selection_msg.img_topic
        threshold=classifier_selection_msg.detection_threshold
        self.startClassifier(classifier, input_img, threshold)


    def startClassifier(self, classifier, input_img, threshold):
        # Check that the requested topic exists and has the expected type
        all_topics = rospy.get_published_topics()
        found_topic = False
        for t in all_topics:
            if (t[0] == input_img) and (t[1] == 'sensor_msgs/Image'):
                found_topic = True
                break
        if (False == found_topic):
            rospy.logerr("Topic %s is not a valid image topic -- not starting classifier", input_img)
            return
            
        # Check if image source has depth_map and pointcloud topics
        depth_map_topic = input_img.rsplit("/")[0] + "/depth_map"
        depth_map_topic = nepi_ros.find_topic(depth_map_topic)
        if depth_map_topic == "":
          depth_map_topic = "None"
          
        pointcloud_topic = input_img.rsplit("/")[0] + "/pointcloud"
        pointcloud_topic = nepi_ros.find_topic(pointcloud_topic)
        if pointcloud_topic == "":
          pointcloud_topic = "None"

        # Validate the requested_detection threshold
        if (threshold < self.MIN_THRESHOLD or threshold > self.MAX_THRESHOLD):
            rospy.logerr("Requested detection threshold (%f) out of range (0.001 - 1.0)", threshold)
            return

        # Check that the requested classifier exists
        if not (classifier in self.classifier_dict.keys()):
            rospy.logerr("Unknown classifier requested: %s", classifier)
            return
        # Stop the current classifier if it is running
        self.stopClassifier()
        time.sleep(1)

        # Update our local status
        self.current_classifier = classifier
        self.current_classifier_classes = self.classifier_dict[classifier]['classes']
        self.current_img_topic = input_img
        self.current_threshold = threshold
 
        # Start the classifier
        model_type = self.classifier_dict[classifier]['type']
        if model_type == "Darknet":
            self.current_model_type = model_type
            self.darknetStartClassifier(classifier=self.current_classifier, input_img=self.current_img_topic, threshold=self.current_threshold, \
            				input_depth_map = depth_map_topic, input_pointcloud = pointcloud_topic)
            

    def stopClassifierCb(self, msg):
        self.stopClassifier()

    def stopClassifier(self):
        if self.current_model_type == "Darknet":
            self.stopDarknetClassifier()
        self.current_model_type = "None"
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED

    def setThresholdCb(self, msg):
        # All we do here is update the current_threshold so that it is up-to-date in status responses
        # and will be saved properly in the config file (on request).
        if (msg.data >= self.MIN_THRESHOLD and msg.data <= self.MAX_THRESHOLD):
            self.current_threshold = msg.data
            if self.current_model_type == "Darknet":
                self.darknet_set_threshold_pub.publish(msg)  # Send to darknet classifier process


    def provideClassifierList(self, req):
        return ImageClassifierListQueryResponse(self.classifier_dict.keys())

    def provideClassifierStatus(self, req):
        # Update the loading progress if necessary
        loading_progress = 0.0
        if self.current_classifier in self.classifier_dict.keys():
            if (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING):
                loading_progress = 1.0
            elif (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING):
                loading_elapsed_s = (rospy.Time.now() - self.classifier_load_start_time).to_sec()
                estimated_load_time_s = self.FIXED_LOADING_START_UP_TIME_S + (self.classifier_dict[self.current_classifier]['size'] / self.ESTIMATED_WEIGHT_LOAD_RATE_BYTES_PER_SECOND)
                if loading_elapsed_s > estimated_load_time_s:
                    loading_progress = 0.95 # Stall at 95%
                else:
                    loading_progress = loading_elapsed_s / estimated_load_time_s
        return [self.current_img_topic, self.current_classifier, str(self.current_classifier_classes), \
                self.classifier_state, loading_progress, self.current_threshold, \
                self.has_depth_map,self.current_depth_map_topic,self.has_pointcloud,self.current_pointcloud_topic]
                               
                
    def setCurrentSettingsAsDefault(self):
        rospy.set_param('~default_classifier', self.current_classifier)
        rospy.set_param('~default_image', self.current_img_topic)
        rospy.set_param('~default_threshold', self.current_threshold)


    def updateFromParamServer(self):
        try:
            default_classifier = rospy.get_param('~default_classifier',"None")
            default_img_topic = rospy.get_param('~default_image',"None")
            default_threshold = rospy.get_param('~default_threshold',0.3)
        except KeyError:
            rospy.loginfo("AI_MGR: Classifier unable to find default parameters... starting up with no classifier running")
            return 

        if default_classifier in self.classifier_dict.keys():
            self.current_classifier = default_classifier
            self.current_classifier_classes = self.classifier_dict[default_classifier]['classes']
            self.current_threshold = default_threshold
            if default_classifier != "None":
                if default_img_topic != "None":
                    check_time = 0
                    sleep_time = 1
                    timeout_s = 20
                    rospy.loginfo("AI_MGR: Will wait for " + str(timeout_s) + " seconds for image topic: " +  default_img_topic)
                    image_topic = nepi_ros.find_topic(default_img_topic)
                    while image_topic == "" and check_time < timeout_s:
                        time.sleep(sleep_time)
                        check_time += sleep_time
                        image_topic = nepi_ros.find_topic(default_img_topic)
                    if check_time < timeout_s:
                        rospy.loginfo('AI_MGR: AI_MGR: Starting classifier with parameters [' + default_classifier + ', ' + default_img_topic + ', ' + str(default_threshold) + ']')
                        self.current_img_topic = default_img_topic
                        self.startClassifier(default_classifier, default_img_topic, default_threshold)

    




    #################
    # Darknet Model Functions

    def darknetFindClassifiers(self):
        classifier_name_list = []
        classifier_size_list = []
        classifier_classes_list = []
        # Try to obtain the path to Darknet models from the system_mgr
        try:
            rospy.wait_for_service('system_storage_folder_query', 10.0)
            system_storage_folder_query = rospy.ServiceProxy('system_storage_folder_query', SystemStorageFolderQuery)
            self.DARKNET_CFG_PATH = os.path.join(system_storage_folder_query('ai_models').folder_path, 'darknet_ros')
        except Exception as e:
            rospy.logwarn("Failed to obtain system ai_models/darknet_ros folder... falling back to " + 'self.DARKNET_CFG_PATH')

        darknet_cfg_path_config_folder = os.path.join(self.DARKNET_CFG_PATH, 'config')
        # Grab the list of all existing darknet cfg files
        self.darknet_cfg_files = glob.glob(os.path.join(darknet_cfg_path_config_folder,'*.yaml'))
        # Remove the ros.yaml file -- that one doesn't represent a selectable trained neural net
        try:
            self.darknet_cfg_files.remove(os.path.join(darknet_cfg_path_config_folder,'ros.yaml'))
        except:
            rospy.logwarn("Unexpected: ros.yaml is missing from the darknet config path %s", darknet_cfg_path_config_folder)

        for f in self.darknet_cfg_files:
            yaml_stream = open(f, 'r')
            # Validate that it is a proper config file and gather weights file size info for load-time estimates
            cfg_dict = yaml.load(yaml_stream)
            #rospy.logwarn("AI_MGR: Debug: " + str(cfg_dict))
            
            yaml_stream.close()
            if ("yolo_model" not in cfg_dict) or ("weight_file" not in cfg_dict["yolo_model"]) or ("name" not in cfg_dict["yolo_model"]["weight_file"]):
                rospy.logerr("Debug: " + str(cfg_dict))
                rospy.logwarn("File does not appear to be a valid A/I model config file: " + f + "... not adding this classifier")
                continue


            classifier_name = os.path.splitext(os.path.basename(f))[0]
            weight_file = os.path.join(self.DARKNET_CFG_PATH, "yolo_network_config", "weights",cfg_dict["yolo_model"]["weight_file"]["name"])
            if not os.path.exists(weight_file):
                rospy.logwarn("Classifier " + classifier_name + " specifies non-existent weights file " + weight_file + "... not adding this classifier")
                continue
            classifier_keys = list(cfg_dict.keys())
            classifier_key = classifier_keys[0]
            classifier_classes_list.append(cfg_dict[classifier_key]['detection_classes']['names'])
            #rospy.logwarn("AI_MGR: classes: " + str(classifier_classes_list))
            classifier_name_list.append(classifier_name)
            classifier_size_list.append(os.path.getsize(weight_file))
        return classifier_name_list, classifier_size_list, classifier_classes_list


    def darknetStartClassifier(self, classifier, input_img, threshold, input_depth_map = "None", input_pointcloud = "None"):
        # Build Darknet new classifier launch command
        launch_cmd_line = [
            "roslaunch", "nepi_darknet_ros", "darknet_ros.launch",
            "namespace:=" + rospy.get_namespace(), 
            "yolo_weights_path:=" + os.path.join(self.DARKNET_CFG_PATH, "yolo_network_config/weights"),
            "yolo_config_path:=" + os.path.join(self.DARKNET_CFG_PATH, "yolo_network_config/cfg"),
            "ros_param_file:=" + os.path.join(self.DARKNET_CFG_PATH, "config/ros.yaml"),
            "network_param_file:=" + os.path.join(self.DARKNET_CFG_PATH, "config", classifier + ".yaml"),
            "input_img:=" + input_img, "input_depth_map:=" + input_depth_map, "input_pointcloud:=" + input_pointcloud,
            "detection_threshold:=" + str(threshold)
        ]
        rospy.loginfo("AI_MGR: Launching Darknet ROS Process: " + str(launch_cmd_line))
        self.darknet_ros_process = subprocess.Popen(launch_cmd_line)
        self.darknet_set_threshold_pub = rospy.Publisher('nepi_darknet_ros/set_threshold', Float32, queue_size=1, latch=True) # Must match the node that gets launched by darknetStartClassifier()

        # Setup Classifier Setup Tracking Progress
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING
        self.classifier_load_start_time = rospy.Time.now()        
        self.darknet_update_sub = rospy.Subscriber('ai_detector_mgr/found_object', ObjectCount, self.darknetUpdateCb) # Resubscribe to found_object so that we know when the classifier is up and running again
            

    def stopDarknetClassifier(self):
        rospy.loginfo("AI_MGR: Stopping classifier")
        if not (None == self.darknet_update_sub):
            self.darknet_update_sub.unregister()
        if not (None == self.darknet_found_object_sub):
            self.darknet_found_object_sub.unregister()
            self.darknet_found_object_sub = None
        if not (None == self.darknet_bounding_boxes_sub):
            self.darknet_bounding_boxes_sub.unregister()
            self.darknet_bounding_boxes_sub = None
        if not (None == self.darknet_ros_process):
            self.darknet_ros_process.terminate()
            self.darknet_ros_process = None
        self.current_classifier = "None"
        self.current_img_topic = "None"
        #self.current_threshold = 0.3

    def darknetUpdateCb(self, msg):
        # Means that darknet is up and running
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING
        if not (None == self.darknet_update_sub):
            self.darknet_update_sub.unregister()


        


 


def main():
    rospy.init_node("ai_detector_manager")
    manager = AIDetectorManager()
    rospy.spin()

if __name__ == '__main__':
    main()
