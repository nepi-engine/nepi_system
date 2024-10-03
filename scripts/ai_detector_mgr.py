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
from nepi_edge_sdk_base import nepi_ais
from nepi_edge_sdk_base import nepi_msg 


from std_msgs.msg import Empty, Float32
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from nepi_ros_interfaces.srv import ImageClassifierListQuery, ImageClassifierListQueryResponse
from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse, SystemStorageFolderQuery
from nepi_ros_interfaces.msg import BoundingBoxes, ObjectCount,ClassifierSelection, StringArray, TargetLocalization


from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF



class AIDetectorManager:
    AI_IF_SEARCH_PATH = '/opt/nepi/ros/share/nepi_ai_ifs'
    AI_MODEL_LIB_PATH = '/mnt/nepi_storage/ai_models/'
    # AI Detection Setttings
    NODE_NAME = "ai_detector_mgr"
    
    MIN_THRESHOLD = 0.001
    MAX_THRESHOLD = 1.0
    FIXED_LOADING_START_UP_TIME_S = 5.0 # Total guess
    ESTIMATED_WEIGHT_LOAD_RATE_BYTES_PER_SECOND = 16000000.0 # Very roughly empirical based on YOLOv3


    classifier_dict = dict()
    class_dict = dict()

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

    found_object_sub = None
   
    classifier_class = None
    classifier_load_start_time = 0
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################
        self.classifier_dict = dict()
        self.class_dict = dict()
        self.classifier_load_start_time = nepi_ros.time_now()
        # Find AI Frameworks
        ais_dict = nepi_ais.getAIsDict(self.AI_IF_SEARCH_PATH)
        nepi_msg.publishMsgWarn(self,"Got ais dict " + str(ais_dict))
        for ai_name in ais_dict.keys():
            ai_dict = ais_dict[ai_name]
            file_name = ai_dict['if_file']
            file_path = ai_dict['if_path']
            module_name = ai_dict['module_name']
            class_name = ai_dict['class_name']
            [success, msg, ai_class] = nepi_ais.importAIClass(file_name,file_path,module_name,class_name)
            if success == False:
                nepi_msg.publishMsgWarn(self,"Failed to import ai framework if file " + file_name)
                break
            else:
                try:
                    class_instance = ai_class(ai_dict,self.node_namespace)
                    time.sleep(1) # Give some time for publishers to set in class init
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to instantiate ai framework class " + class_name + " " + str(e))
                    break
                try:
                    models_dict = class_instance.getModelsDict()
                    for model_name in models_dict.keys():
                        model_dict = models_dict[model_name]
                        model_dict['type'] = ai_name
                        model_dict['active'] = True
                        self.classifier_dict[model_name] = model_dict
                        self.class_dict[model_name] = class_instance
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to get models from class " + class_name)
                    break
                
                if (len(model_dict.keys()) < 1):
                    nepi_msg.publishMsgWarn(self,"No classiers identified for this system at " + file_path)
        #nepi_msg.publishMsgWarn(self,"Storing classifier dict in param server: " + str(self.classifier_dict))
        nepi_ros.set_param(self,'~classifier_dict', self.classifier_dict)

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
        #namespace = nepi_ros.get_node_namespace()
        self.updateFromParamServer()
        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################

    def startClassifierCb(self, classifier_selection_msg):
        classifier_name=classifier_selection_msg.classifier
        input_img=classifier_selection_msg.img_topic
        threshold=classifier_selection_msg.detection_threshold
        self.startClassifier(classifier_name, input_img, threshold)


    def startClassifier(self, classifier_name, input_img, threshold):
        # Check that the requested topic exists and has the expected type
        all_topics = nepi_ros.get_published_topics()
        found_topic = False
        for t in all_topics:
            if (t[0] == input_img) and (t[1] == 'sensor_msgs/Image'):
                found_topic = True
                break
        if (False == found_topic):
            nepi_msg.publishMsgErr(self,"Topic " + input_img + " is not a valid image topic -- not starting classifier")
            return
            
        # Validate the requested_detection threshold
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD

        # Check that the requested classifier exists
        if not (classifier_name in self.classifier_dict.keys()):
            nepi_msg.publishMsgErr(self,"Unknown classifier requested: " + classifier_name)
            return
        # Stop the current classifier if it is running
        self.stopClassifier()
        time.sleep(1)

        # Update our local status
        self.current_classifier = classifier_name
        self.current_classifier_classes = self.classifier_dict[classifier_name]['classes']
        self.current_img_topic = input_img
        self.current_threshold = threshold
 
        # Start the classifier
        classifier = self.classifier_dict[classifier_name]['name']
        classifier_class = self.class_dict[classifier_name]
        self.classifier_class = classifier_class
        self.classifier_load_start_time = nepi_ros.time_now()
        self.classifier_class.startClassifier(classifier=classifier, input_img=self.current_img_topic, threshold=self.current_threshold)
        if self.found_object_sub is not None:
            self.found_object_sub = rospy.Subscriber('ai_detector_mgr/found_object', ObjectCount, self.UpdateCb) # Resubscribe to found_object so that we know when the classifier is up and running again
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING        
        self.update_state_sub = rospy.Subscriber('ai_detector_mgr/found_object', ObjectCount, self.stateUpdateCb) # Resubscribe to found_object so that we know when the classifier is up and running again



    def stateUpdateCb(self, msg):
        # Means that darknet is up and running
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING
        if not (None == self.update_state_sub):
            self.update_state_sub.unregister()


    def stopClassifierCb(self, msg):
        self.stopClassifier()
        
    def stopClassifier(self):
        if self.classifier_class != None:
            self.classifier_class.stopClassifier()
            if not (None == self.update_state_sub):
                self.update_state_sub.unregister()
            self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED
            self.current_threshold = None

    def setThresholdCb(self, msg):
        # All we do here is update the current_threshold so that it is up-to-date in status responses
        # and will be saved properly in the config file (on request).
        threshold = msg.data
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD
        self.current_threshold = msg.data
        if self.classifier_class != None:
            self.classifier_class.updateClassifierThreshold(self.current_threshold)  # Send to classifier process


    def provideClassifierList(self, req):
        return ImageClassifierListQueryResponse(self.classifier_dict.keys())

    def provideClassifierStatus(self, req):
        # Update the loading progress if necessary
        loading_progress = 0.0
        if self.classifier_class is not None:
            if self.current_classifier in self.classifier_dict.keys():
                if (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING):
                    loading_progress = 1.0
                elif (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING):
                    loading_elapsed_s = (nepi_ros.time_now() - self.classifier_load_start_time).to_sec()
                    estimated_load_time_s = self.FIXED_LOADING_START_UP_TIME_S + (self.classifier_dict[self.current_classifier]['size'] / self.ESTIMATED_WEIGHT_LOAD_RATE_BYTES_PER_SECOND)
                    if loading_elapsed_s > estimated_load_time_s:
                        loading_progress = .95
                    else:
                        loading_progress = loading_elapsed_s / estimated_load_time_s
        return [self.current_img_topic, self.current_classifier, str(self.current_classifier_classes), \
                self.classifier_state, loading_progress, self.current_threshold, \
                self.has_depth_map,self.current_depth_map_topic,self.has_pointcloud,self.current_pointcloud_topic]




                
    def setCurrentSettingsAsDefault(self):
        nepi_ros.set_param(self,'~default_classifier', self.current_classifier)
        nepi_ros.set_param(self,'~default_image', self.current_img_topic)
        nepi_ros.set_param(self,'~default_threshold', self.current_threshold)


    def updateFromParamServer(self):
        try:
            default_classifier = nepi_ros.get_param(self,'~default_classifier',"None")
            default_img_topic = nepi_ros.get_param(self,'~default_image',"None")
            default_threshold = nepi_ros.get_param(self,'~default_threshold',0.3)
        except KeyError:
            nepi_msg.publishMsgInfo(self,"Classifier unable to find default parameters... starting up with no classifier running")
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
                    nepi_msg.publishMsgInfo(self,"Will wait for " + str(timeout_s) + " seconds for image topic: " +  default_img_topic)
                    image_topic = nepi_ros.find_topic(default_img_topic)
                    while image_topic == "" and check_time < timeout_s:
                        time.sleep(sleep_time)
                        check_time += sleep_time
                        image_topic = nepi_ros.find_topic(default_img_topic)
                    if check_time < timeout_s:
                        nepi_msg.publishMsgInfo(self,'AI_MGR: AI_MGR: Starting classifier with parameters [' + default_classifier + ', ' + default_img_topic + ', ' + str(default_threshold) + ']')
                        self.current_img_topic = default_img_topic
                        self.startClassifier(default_classifier, default_img_topic, default_threshold)

    

if __name__ == '__main__':
    AIDetectorManager()
