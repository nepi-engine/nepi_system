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
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_ais
from nepi_edge_sdk_base import nepi_msg 
from nepi_edge_sdk_base import nepi_img


from std_msgs.msg import Empty, Float32
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from nepi_ros_interfaces.msg import UpdateState, AiFrameworksStatus
from nepi_ros_interfaces.srv import ImageClassifierListQuery, ImageClassifierListQueryResponse
from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse, SystemStorageFolderQuery
from nepi_ros_interfaces.msg import BoundingBoxes, ObjectCount,ClassifierSelection


from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_edge_sdk_base.save_data_if import SaveDataIF


class AIDetectorManager:
    AI_IF_FOLDER_PATH = '/opt/nepi/ros/share/nepi_ai_ifs'
    AI_MODEL_LIB_PATH = '/mnt/nepi_storage/ai_models/'
    # AI Detection Setttings
    NODE_NAME = "ai_detector_mgr"
    
    MIN_THRESHOLD = 0.001
    MAX_THRESHOLD = 1.0
    

    data_products = ['bounding_boxes','detection_image']

    init_ais_dict = dict()
    init_models_dict = dict()
    class_dict = dict()

    current_classifier = "None"
    current_classifier_classes = []
    current_model_type = "None"
    current_img_topic = "None"
    has_depth_map = False
    depth_map_topic = "None"
    has_pointcloud = False
    pointcloud_topic = "None"
    current_threshold = 0.3
    classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED
    classifier_load_start_time = None

    found_object_sub = None
   
    classifier_class = None
    classifier_load_start_time = 0
    save_cfg_if = None
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "ai_detector_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### MGR NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################
        self.models_dict = dict()
        self.class_dict = dict()
        self.classifier_load_start_time = nepi_ros.time_now()

        # Create a message image to publish when not running
        message = "DETECTOR_NOT_RUNNING"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_message_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        # Initialize apps_mgr param server
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, 
                                    paramsModifiedCallback=self.updateFromParamServer)
        self.save_cfg_if.userReset()
        time.sleep(3)
        ## Find AI Frameworks
        # Get ai framework dict form param server and update
        ais_dict = nepi_ais.getAIsDict(self.AI_IF_FOLDER_PATH)
        #nepi_msg.publishMsgWarn(self,"Got ais dict " + str(ais_dict))
        self.init_ais_dict = nepi_ros.get_param(self,'~ais_dict', ais_dict)
        try:
            self.init_ais_dict = nepi_ais.updateAIsDict(self.AI_IF_FOLDER_PATH,self.init_ais_dict)
        except:
            nepi_msg.publishMsgWarn(self,"Got bad ais_dict from param server so resetting")
            self.init_ais_dict = ais_dict
        nepi_ros.set_param(self,'~ais_dict', self.init_ais_dict)
        #nepi_msg.publishMsgWarn(self,"Got updated ais dict from param server " + str(self.init_ais_dict))
        for ai_name in self.init_ais_dict.keys():
            ai_dict = self.init_ais_dict[ai_name]
            nepi_msg.publishMsgInfo(self,"Processing ais dict for ai name " + ai_name + " " + str(ai_dict))
            if ai_dict['active']:
                nepi_msg.publishMsgInfo(self,"Updating ai dict for framework: " + str(ai_name))
                file_name = ai_dict['if_file']
                file_path = ai_dict['if_path']
                module_name = ai_dict['module_name']
                class_name = ai_dict['class_name']
                [success, msg, ai_class] = nepi_ais.importAIClass(file_name,file_path,module_name,class_name)
                if success == False:
                    nepi_msg.publishMsgWarn(self,"Failed to import ai framework if file " + file_name)
                    break
                else:
                    success = False
                    try:
                        nepi_msg.publishMsgInfo(self,"Instantiating IF class for framework type: " + str(ai_name))
                        pub_sub_namespace = os.path.join(self.base_namespace, self.node_name)
                        class_instance = ai_class(ai_dict,pub_sub_namespace,self.AI_MODEL_LIB_PATH)
                        success = True
                        time.sleep(1) # Give some time for publishers to set in class init
                    except Exception as e:
                        nepi_msg.publishMsgWarn(self,"Failed to instantiate ai framework class " + class_name + " " + str(e))

                    if success:
                        try:
                            models_dict = class_instance.getModelsDict()
                            for model_name in models_dict.keys():
                                model_dict = models_dict[model_name]
                                model_dict['type'] = ai_name
                                model_dict['active'] = True
                                self.models_dict[model_name] = model_dict
                                self.class_dict[model_name] = class_instance
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Failed to get models from class " + class_name + " " + str(e))
                            break
                        
                        if (len(model_dict.keys()) < 1):
                            nepi_msg.publishMsgWarn(self,"No classiers identified for this system at " + file_path)
                        else:
                            nepi_msg.publishMsgInfo(self,"Got models for framework type: " + str(ai_name) + " from param server " + str(self.models_dict.keys()))
        # Update models dict against imported param dict
        #nepi_msg.publishMsgWarn(self,"Got models dict from nepi_ais call " + str(self.models_dict))
        self.init_models_dict = nepi_ros.get_param(self,'~models_dict', self.models_dict)
        nepi_ros.set_param(self,'~models_dict', self.init_models_dict)
        purge_list = []
        for model_name in self.init_models_dict.keys():
            if model_name not in self.models_dict.keys():
                purge_list.append(model_name)
        for model_name in purge_list:
            del self.init_models_dict[model_name]
        for model_name in self.models_dict.keys():
            if model_name not in self.init_models_dict.keys():
                self.init_models_dict[model_name] = self.models_dict[model_name]
        nepi_ros.set_param(self,'~models_dict', self.init_models_dict)
        #nepi_msg.publishMsgWarn(self,"Storing classifier dict in param server: " + str(self.init_models_dict))
       
        # Set up save data and save config services ########################################################
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)


        # Setup Node Services
        rospy.Service('~img_classifier_list_query', ImageClassifierListQuery, self.provideClassifierList)
        rospy.Service('~img_classifier_status_query', ImageClassifierStatusQuery, self.provideClassifierStatus)
        # Setup Constant Node Subscribers
        rospy.Subscriber('~start_classifier', ClassifierSelection, self.startClassifierCb)
        rospy.Subscriber('~stop_classifier', Empty, self.stopClassifierCb)
        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb) 

        ## Mgr ROS Setup 
        #mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
        #mgr_reset_sub = rospy.Subscriber('~refresh_ais', Empty, self.refreshCb, queue_size = 10)

        self.pub_sub_namespace = self.base_namespace + self.node_name
        SOURCE_IMAGE_TOPIC = self.pub_sub_namespace + "/source_image"
        self.source_image_pub = rospy.Publisher(SOURCE_IMAGE_TOPIC, Image, queue_size=1, latch=True)
        DETECTION_IMAGE_TOPIC = self.pub_sub_namespace + "/detection_image"
        self.detection_image_pub = rospy.Publisher(DETECTION_IMAGE_TOPIC, Image,queue_size=1, latch=True)
        rospy.Subscriber(DETECTION_IMAGE_TOPIC, Image, self.detectionImageCb, queue_size = 1)
        BOUNDING_BOXES_TOPIC = self.pub_sub_namespace + "/bounding_boxes"
        rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, self.boundingBoxesCb, queue_size = 1)
        time.sleep(1)

        # Framework Management Scubscirbers
        rospy.Subscriber('~enable_all_ais', Empty, self.enableAllFwsCb, queue_size = 10)
        rospy.Subscriber('~disable_all_ais', Empty, self.disableAllFwsCb, queue_size = 10)
        rospy.Subscriber('~update_ais_state', UpdateState, self.updateFwStateCb)
        #Model Management Scubscirbers
        rospy.Subscriber('~enable_all_models', Empty, self.enableAllModelsCb, queue_size = 10)
        rospy.Subscriber('~disable_all_models', Empty, self.disableAllModelsCb, queue_size = 10)
        rospy.Subscriber('~update_model_state', UpdateState, self.updateModelStateCb)
        # Create status pub

        self.aif_status_pub = rospy.Publisher("~status", AiFrameworksStatus, queue_size=1, latch=True)
        time.sleep(1)
        rospy.Timer(rospy.Duration(0.5), self.statusPublishCb)


        # Load default params
        self.updateFromParamServer()
        self.saveEnabledSettings() # Save config
        self.ros_message_img.header.stamp = nepi_ros.time_now()
        self.source_image_pub.publish(self.ros_message_img)
        self.detection_image_pub.publish(self.ros_message_img)
        self.publish_status()
        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


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
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        if default_classifier in models_dict.keys():
            self.current_classifier = default_classifier
            self.current_classifier_classes = models_dict[default_classifier]['classes']
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

    def detectionImageCb(self,img_in_msg):
        if self.classifier_state == "Running":
            data_product = 'detection_image'
            ros_timestamp = img_in_msg.header.stamp
            nepi_save.save_ros_img2file(self,data_product,img_in_msg,ros_timestamp)


    def boundingBoxesCb(self,bbs_msg):
        data_product = 'bounding_boxes'
        ros_timestamp = bbs_msg.header.stamp
        bbs_dict = dict()
        bbs_dict['timestamp'] =  nepi_ros.get_datetime_str_from_stamp(bbs_msg.header.stamp)
        bbs_dict['image_topic'] = bbs_msg.image_topic
        bbs_dict['image_height'] = bbs_msg.image_height
        bbs_dict['image_width'] = bbs_msg.image_width

        bb_list = []
        for ind, bb_msg in enumerate(bbs_msg.bounding_boxes):
            bb_dict = dict()
            bb_dict['class'] = bb_msg.Class
            bb_dict['id'] = bb_msg.id
            bb_dict['uid'] = bb_msg.uid
            bb_dict['probability'] = bb_msg.probability
            bb_dict['xmin'] = bb_msg.xmin
            bb_dict['ymin'] = bb_msg.ymin
            bb_dict['xmax'] = bb_msg.xmax
            bb_dict['ymax'] = bb_msg.ymax
            bb_dict['area_pixels'] = bb_msg.area_pixels
            bb_dict['area_ratio'] = bb_msg.area_ratio
            bb_list.append(bb_dict)
        bbs_dict['bounding_boxes'] = bb_list
        nepi_save.save_dict2file(self,data_product,bbs_dict,ros_timestamp)


    def statusPublishCb(self,timer):
        self.publish_status()


    def publish_status(self):
        ais_dict = nepi_ros.get_param(self,"~ais_dict",self.init_ais_dict)
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        status_msg = AiFrameworksStatus()
        status_msg.ai_frameworks = nepi_ais.getAIsSortedList(ais_dict)
        status_msg.active_ai_frameworks = nepi_ais.getAIsActiveSortedList(ais_dict)
        status_msg.ai_models = nepi_ais.getModelsSortedList(models_dict)
        status_msg.active_ai_models = nepi_ais.getModelsActiveSortedList(models_dict)
        if not nepi_ros.is_shutdown():
            self.aif_status_pub.publish(status_msg)


    def startClassifierCb(self, classifier_selection_msg):
        nepi_msg.publishMsgInfo(self,"Got start classifier msg: " + str(classifier_selection_msg))
        classifier_name=classifier_selection_msg.classifier
        source_img_topic=classifier_selection_msg.img_topic
        threshold=classifier_selection_msg.detection_threshold
        if classifier_name != 'None':
            self.startClassifier(classifier_name, source_img_topic, threshold)



    def startClassifier(self, classifier_name, source_img_topic, threshold):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        # Check that the requested topic exists and has the expected type
        all_topics = nepi_ros.get_published_topics()
        found_topic = False
        for t in all_topics:
            if (t[0] == source_img_topic) and (t[1] == 'sensor_msgs/Image'):
                found_topic = True
                break
        if (False == found_topic):
            nepi_msg.publishMsgErr(self,"Topic " + source_img_topic + " is not a valid image topic -- not starting classifier")
            return
            
        # Validate the requested_detection threshold
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD

        # Check that the requested classifier exists
        
        if not (classifier_name in models_dict.keys()):
            nepi_msg.publishMsgErr(self,"Unknown classifier requested: " + classifier_name)
            return
        # Stop the current classifier if it is running
        self.stopClassifier()
        time.sleep(1)

        # Update our local status
        self.current_classifier = classifier_name
        self.current_classifier_classes = models_dict[classifier_name]['classes']
        self.current_img_topic = source_img_topic
        self.current_threshold = threshold
 
        # Start the classifier
        
        classifier = models_dict[classifier_name]['name']
        classifier_class = self.class_dict[classifier_name]
        self.classifier_class = classifier_class
        self.classifier_load_start_time = nepi_ros.time_now()
        nepi_msg.publishMsgInfo(self,"Starting classifier " + classifier_name + " with classifier " + classifier + " with image " + self.current_img_topic)
        self.classifier_class.startClassifier(classifier=classifier, source_img_topic=self.current_img_topic, threshold=self.current_threshold)
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
            time.sleep(1)
            self.classifier_class = None
            if not (None == self.update_state_sub):
                self.update_state_sub.unregister()
            self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED
            #self.current_threshold = None
            time.sleep(1)
            self.ros_message_img.header.stamp = nepi_ros.time_now()
            self.source_image_pub.publish(self.ros_message_img)
            self.detection_image_pub.publish(self.ros_message_img)

    def setThresholdCb(self, msg):
        # All we do here is update the current_threshold so that it is up-to-date in status responses
        # and will be saved properly in the config file (on request).  The actual threshold update is
        # done by ai framework node
        threshold = msg.data
        if (threshold < self.MIN_THRESHOLD):
            threshold = self.MIN_THRESHOLD
        elif (threshold > self.MAX_THRESHOLD):
            threshold = self.MAX_THRESHOLD
        self.current_threshold = threshold

    def saveEnabledSettings(self):
        # Clear classifier and image setting
        nepi_ros.set_param(self,'~default_classifier', "None")
        nepi_ros.set_param(self,'~default_image', "None")
        # Save framework and model dictionaries
        self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for drvt time
        # Reset classifier and image settings
        nepi_ros.set_param(self,'~default_classifier', self.current_classifier)
        nepi_ros.set_param(self,'~default_image', self.current_img_topic)



    def enableAllFwsCb(self,msg):
        ais_dict = nepi_ros.get_param(self,"~ais_dict",self.init_ais_dict)
        ais_dict = nepi_ais.activateAllFws(ais_dict)
        nepi_ros.set_param(self,"~ais_dict",ais_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()

    def disableAllFwsCb(self,msg):
        ais_dict = nepi_ros.get_param(self,"~ais_dict",self.init_ais_dict)
        ais_dict = nepi_ais.disableAllFws(ais_dict)
        nepi_ros.set_param(self,"~ais_dict",ais_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()

    def updateFwStateCb(self,msg):
        nepi_msg.publishMsgInfo(self,str(msg))
        ai_name = msg.name
        new_active_state = msg.active_state
        ais_dict = nepi_ros.get_param(self,"~ais_dict",self.init_ais_dict)
        if ai_name in ais_dict.keys():
            app = ais_dict[ai_name]
            active_state = app['active']
            if new_active_state != active_state:
                if new_active_state == True:
                    ais_dict = nepi_ais.activateFw(ai_name,ais_dict)
                else:
                    ais_dict = nepi_ais.disableFw(ai_name,ais_dict)
        nepi_ros.set_param(self,"~ais_dict",ais_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()

    def enableAllModelsCb(self,msg):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        models_dict = nepi_ais.activateAllModels(models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()

    def disableAllModelsCb(self,msg):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        models_dict = nepi_ais.disableAllModels(models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()

    def updateModelStateCb(self,msg):
        nepi_msg.publishMsgInfo(self,str(msg))
        model_name = msg.name
        new_active_state = msg.active_state
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        if model_name in models_dict.keys():
            app = models_dict[model_name]
            active_state = app['active']
            if new_active_state != active_state:
                if new_active_state == True:
                    models_dict = nepi_ais.activateModel(model_name,models_dict)
                else:
                    models_dict = nepi_ais.disableModel(model_name,models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveEnabledSettings() # Save config
        self.publish_status()


    def provideClassifierList(self, req):
        ais_dict = nepi_ros.get_param(self,"~ais_dict",self.init_ais_dict)
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        #nepi_msg.publishMsgWarn(self,"Returning ais dict " + str(ais_dict))
        #nepi_msg.publishMsgWarn(self,"Returning ais sorted list " + str(nepi_ais.getAIsSortedList(ais_dict)))
        #nepi_msg.publishMsgWarn(self,"Returning models dict " + str(models_dict))
        #nepi_msg.publishMsgWarn(self,"Returning models sorted list " + str(nepi_ais.getModelsSortedList(models_dict)))
        response = ImageClassifierListQueryResponse()
        response.ai_frameworks = nepi_ais.getAIsSortedList(ais_dict)
        response.active_ai_frameworks = nepi_ais.getAIsActiveSortedList(ais_dict)
        response.models = nepi_ais.getModelsSortedList(models_dict)
        response.active_models = nepi_ais.getModelsActiveSortedList(models_dict)
        return response

    def provideClassifierStatus(self, req):
        # Update the loading progress if necessary
        loading_progress = 0.0
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        if self.classifier_class is not None:
            if self.current_classifier in models_dict.keys():
                if (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING):
                    loading_progress = 1.0
                elif (self.classifier_state == ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING):
                    loading_elapsed_s = (nepi_ros.time_now() - self.classifier_load_start_time).to_sec()
                    estimated_load_time_s = models_dict[self.current_classifier]['load_time'] 
                    if loading_elapsed_s > estimated_load_time_s:
                        loading_progress = .95
                    else:
                        loading_progress = loading_elapsed_s / estimated_load_time_s
        if (self.classifier_state != ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING):
            self.ros_message_img.header.stamp = nepi_ros.time_now()
            self.detection_image_pub.publish(self.ros_message_img)
            self.source_image_pub.publish(self.ros_message_img)

        # Look for Depth Map
        if self.current_img_topic != "None":
            depth_map_topic = self.current_img_topic.rsplit('/',1)[0] + "/depth_map"
            depth_map_topic = nepi_ros.find_topic(depth_map_topic)
            if depth_map_topic == "":
                depth_map_topic = "None"
                self.has_depth_map = False
            else:
                self.has_depth_map = True
                self.depth_map_topic = depth_map_topic
            # check for pointdcloud
            pointcloud_topic = self.current_img_topic.rsplit('/',1)[0] + "/pointcloud"
            pointcloud_topic = nepi_ros.find_topic(pointcloud_topic)
            if pointcloud_topic == "":
                pointcloud_topic = "None"
                self.has_pointcloud = False
            else:
                self.has_pointcloud = True
                self.pointcloud_topic = pointcloud_topic     
        else:
            depth_map_topic = "None"
            self.has_depth_map = False
            pointcloud_topic = "None"
            self.has_pointcloud = False

        return [self.current_img_topic, self.current_classifier, self.current_classifier_classes, \
                self.classifier_state, loading_progress, self.current_threshold, \
                self.has_depth_map,self.depth_map_topic,self.has_pointcloud,self.pointcloud_topic]




                
 

    

if __name__ == '__main__':
    AIDetectorManager()
