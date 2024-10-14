#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import sys
import subprocess
import time
from wsdiscovery.discovery import ThreadedWSDiscovery as WSDiscovery
import urllib.parse

import datetime
from datetime import timezone
import requests
from xml.etree import ElementTree as ET

# DON'T USE SaveCfgIF IN THIS CLASS -- SEE WARNING BELOW
#from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_drv

from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyResponse
from nepi_ros_interfaces.msg import DriversStatus
from nepi_ros_interfaces.msg import OnvifDeviceCfg, OnvifDeviceStatus
from nepi_ros_interfaces.srv import OnvifDeviceListQuery, OnvifDeviceListQueryResponse
from nepi_ros_interfaces.srv import OnvifDeviceCfgUpdate, OnvifDeviceCfgUpdateResponse
from nepi_ros_interfaces.srv import OnvifDeviceCfgDelete, OnvifDeviceCfgDeleteResponse
from nepi_ros_interfaces.srv import OnvifDeviceDriverListQuery, OnvifDeviceDriverListQueryResponse

MGR_NAME = 'ONVIF Manager' # Use in display menus
FILE_TYPE = 'MANAGER'
MGR_DICT = dict(
    description = 'Application for managing onvif device connection credentials',
    pkg_name = 'nepi_managers',
    group_name = 'DRIVER',
    config_file = 'onvif_mgr.yaml',
    app_file = 'onvif_mgr.py',
    node_name = 'onvif_mgr'
)
RUI_DICT = dict(
    rui_menu_name = "ONVIF Device Manager", # RUI menu name or "None" if no rui support
    rui_files = ['NepiDriverOnvif.js'],
    rui_main_file = "NepiDriverOnvif.js",
    rui_main_class = "OnvifMgr"
)



DRIVERS_PATH = "/opt/nepi/ros/lib/nepi_drivers/"

class ONVIFMgr:

  DEFAULT_NEPI_CONFIG_PATH = "/opt/nepi/ros/etc"
  WSDL_FOLDER = os.path.join(DEFAULT_NEPI_CONFIG_PATH, "onvif/wsdl/")

 
  DEFAULT_DISCOVERY_INTERVAL_S = 10.0
  
  ONVIF_SCOPE_NVT_ID = 'Network_Video_Transmitter'
  ONVIF_SCOPE_NVT_ALT_ID = 'NetworkVideoTransmitter' # ONVIF spec. says this name is legal for NVT, too
  ONVIF_SCOPE_PTZ_ID = 'ptz'

  device_name_dict = dict()
  drvs_dict = dict()
  drivers_files = []
  active_drivers_list = []
  idx_drivers_dict = []
  ptx_drivers_dict = []

  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "onvif_app" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    self.discovery_interval_s = nepi_ros.get_param(self,'~discovery_interval_s', self.DEFAULT_DISCOVERY_INTERVAL_S)
    self.autosave_cfg_changes = nepi_ros.get_param(self,'~autosave_cfg_changes', True)

    self.init_clear_disc = nepi_ros.get_param(self,"~clear_discovery",False)
    nepi_ros.get_param(self,"~clear_discovery",self.init_clear_disc)
    self.detected_onvifs = {}

    self.wsd = WSDiscovery()
    self.wsd.start()
    detected_services = self.wsd.searchServices(timeout=1)

    self.configured_onvifs = nepi_ros.get_param(self,'~onvif_devices', {})
    nepi_msg.publishMsgInfo(self,"Starting device dict from param server: " + str(self.configured_onvifs) )
    # Get drv drivers database
    self.drvs_dict = nepi_drv.getDriversDict(DRIVERS_PATH)
    self.drivers_files = nepi_drv.getDriverFilesList(DRIVERS_PATH)
    # Get active drivers list from nepi_mgr_drivers
    NEPI_DRIVERS_STATUS_TOPIC = self.base_namespace + 'drivers_mgr/status'
    nepi_msg.publishMsgInfo(self,'Waiting for driver_mgr status message')
    nepi_ros.wait_for_topic(NEPI_DRIVERS_STATUS_TOPIC)
    rospy.Subscriber(NEPI_DRIVERS_STATUS_TOPIC, DriversStatus, self.driversStatusCb)
    nepi_msg.publishMsgInfo(self,'Waiting for active drivers list')
    while (len(self.active_drivers_list) == 0) and not nepi_ros.is_shutdown():
      time.sleep(1)


    # Iterate through these configured devices fixing invalid characters in the node base name
    for uuid in self.configured_onvifs:
      self.configured_onvifs[uuid]['node_base_name'] = self.configured_onvifs[uuid]['node_base_name'].replace(' ','_').replace('-','_')

    #### WARNING ####
    # Do not use topics in this class, only services... somehow just the execution of any subscriber
    # callback breaks the ONVIF discovery mechanism, even if the callback does nothing at all and 
    # even if the WSDiscovery object is destroyed and recreated afterwards. Very weird, but very repeatable.
    #rospy.Subscriber('~set_device_cfg', OnvifDeviceCfg, self.setDeviceCfgHandler, queue_size=5)
    
    # This WARNING extends to topics that are part of an included interface (e.g., SaveCfgIF) -- 
    # can't use those interfaces, instead must manage config. file saving ourselves!
    # self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)
    #### END WARNING ####

    rospy.Service('~set_device_cfg', OnvifDeviceCfgUpdate, self.updateDeviceCfgHandler)
    rospy.Service('~delete_device_cfg', OnvifDeviceCfgDelete, self.deleteDeviceCfgHandler)
    rospy.Service('~device_list_query', OnvifDeviceListQuery, self.provideDeviceList)
    rospy.Service('~resync_onvif_device_clocks', Empty, self.resyncOnvifDeviceClocks)
    rospy.Service('~device_driver_list_query', OnvifDeviceDriverListQuery, self.provideDeviceDriverList)

    rospy.Subscriber('~allow_discovery_clearing', Bool, self.clearDiscCb, queue_size = 10)
    
    # Must handle our own store params rather than offloading to SaveCfgIF per WARNING above
    self.store_params_publisher = rospy.Publisher('store_params', String, queue_size=1)

    dummy = None
    self.runDiscovery(dummy) # Run discovery immediately. It will re-run itself via a timer after this first one
    #########################################################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    #Set up node shutdown
    nepi_ros.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_ros.spin()
    #########################################################


  def clearDiscCb(self,msg):
    clear = msg.data
    nepi_ros.set_param(self,"~clear_discovery",clear)
    if self.autosave_cfg_changes is True:
      nepi_msg.publishMsgInfo(self,'Auto-saving updated config')
      self.setCurrentSettingsAsDefault()
      self.store_params_publisher.publish(nepi_ros.get_node_namespace())

  def driversStatusCb(self,msg):
    # First check if drv driver database needs updating
    drvs_dict = self.drvs_dict
    drivers_files = nepi_drv.getDriverFilesList(DRIVERS_PATH)
    need_update = self.drivers_files != drivers_files
    if need_update:
      nepi_msg.publishMsgInfo(self,"Need to Update Drv Database")
      drvs_dict = nepi_drv.updateDriversDict(DRIVERS_PATH,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    #ln = sys._getframe().f_lineno ; self.printND('Info',ln)
    self.drivers_files = drivers_files
    # Next update available drivers base on active drivers
    active_drivers_list = msg.drivers_active_list
    for drv_name in self.drvs_dict.keys():
      active = False
      if drv_name in active_drivers_list:
        active = True
      self.drvs_dict[drv_name]['active'] = active
    active_drvs_dict = nepi_drv.getDriversByActive(self.drvs_dict)
    idx_drivers_dict = dict()
    ptx_drivers_dict = dict()
    for drv_name in active_drvs_dict.keys():
      drv_dict = active_drvs_dict[drv_name]
      group = drv_dict['group']
      group_id = drv_dict['group_id']
      if group == "IDX" and group_id == "ONVIF":
        idx_drivers_dict[drv_name] = drv_dict
      elif group == "PTX" and group_id == "ONVIF":
        ptx_drivers_dict[drv_name] = drv_dict
    self.idx_drivers_dict = idx_drivers_dict
    self.ptx_drivers_dict = ptx_drivers_dict
    self.active_drivers_list = active_drivers_list
    #nepi_msg.publishMsgWarn(self,str(active_drivers_list))
    #nepi_msg.publishMsgWarn(self,str(self.active_drivers_list))



  def updateDeviceCfgHandler(self, req):
    device_cfg = req.cfg
    if device_cfg.uuid == "Searching":
      return OnvifDeviceCfgUpdateResponse(success = True)
    else:
      uuid = device_cfg.uuid.upper()
      
      if uuid in self.detected_onvifs:
        detected_device = self.detected_onvifs[uuid]
      else:
        nepi_msg.publishMsgInfo(self,'Setting configuration for as-yet undetected device ' + str(uuid))
        detected_device = None

      # Make sure the node_base_name is legal ROS
      legal_base_name = device_cfg.node_base_name.replace(' ', '_').replace('-','_')

      # Make sure the specified drivers are known
      if device_cfg.idx_driver == "":
          idx_drv_dict = dict()
      else:
        if device_cfg.idx_driver not in self.idx_drivers_dict.keys():
          nepi_msg.publishMsgWarn(self,'Unknown idx driver(s) specified ( ' + str(device_cfg.idx_driver) + ' )... will not update config')
          return OnvifDeviceCfgUpdateResponse(success = False)
        else:
          idx_drv_dict = self.idx_drivers_dict[device_cfg.idx_driver]

      if device_cfg.ptx_driver == "":
          ptx_drv_dict = dict()
      else:
        if device_cfg.ptx_driver not in self.ptx_drivers_dict.keys():
          nepi_msg.publishMsgWarn(self,'Unknown ptx driver(s) specified ( ' + str(device_cfg.ptx_driver) + ' )... will not update config')
          return OnvifDeviceCfgUpdateResponse(success = False)
        else:
          ptx_drv_dict = self.ptx_drivers_dict[device_cfg.ptx_driver]        

      updated_cfg = {
        'device_name' : device_cfg.device_name,
        'username' : device_cfg.username,
        'password' : device_cfg.password,
        'node_base_name' : legal_base_name,
        'idx_enabled' : device_cfg.idx_enabled,
        'ptx_enabled' : device_cfg.ptx_enabled,
        'idx_driver' : device_cfg.idx_driver,
        'idx_drv_dict': idx_drv_dict,
        'ptx_driver' : device_cfg.ptx_driver,
        'ptx_drv_dict': ptx_drv_dict
      }
      self.configured_onvifs[uuid] = updated_cfg
      nepi_ros.set_param(self,'~onvif_devices', self.configured_onvifs)
      # Now remove the device from the list of detected devices so that it can be rediscovered and properly connected and nodes launched
      # using this updated config. If nodes are already running, stop them (to restart as newly-configured) and warn user
      if detected_device:
        if (detected_device['idx_subproc'] is not None) or (detected_device['ptx_subproc'] is not None):
          nepi_msg.publishMsgWarn(self,'Config. for ' + str(uuid) + ' is updated, will restart any already-running nodes')
          self.stopAndPurgeNodes(uuid)
        
        self.detected_onvifs.pop(uuid)

      # Must handle our own saving since topics don't work in this class (see WARNING above)
      if self.autosave_cfg_changes is True:
        nepi_msg.publishMsgInfo(self,'Auto-saving updated config')
        self.setCurrentSettingsAsDefault()
        self.store_params_publisher.publish(nepi_ros.get_node_namespace())

      return OnvifDeviceCfgUpdateResponse(success = True)

  def deleteDeviceCfgHandler(self, req):
    uuid = req.device_uuid.upper()
    if uuid not in self.configured_onvifs:
      nepi_msg.publishMsgWarn(self,"Device is not configured... ignoring request to delete configuration")
      return OnvifDeviceCfgDeleteResponse(success = False)
    
    self.configured_onvifs.pop(uuid)
    nepi_ros.set_param(self,'~onvif_devices', self.configured_onvifs)
    # And clean it up if this device is currently detected and running
    if uuid in self.detected_onvifs:
      self.stopAndPurgeNodes(uuid)
      self.detected_onvifs.pop(uuid)

    # Must handle our own saving since topics don't work in this class (see WARNING above)
    if self.autosave_cfg_changes is True:
      nepi_msg.publishMsgInfo(self,'Auto-saving deleted config')
      self.setCurrentSettingsAsDefault()
      self.store_params_publisher.publish(nepi_ros.get_node_namespace())

    return True

  def provideDeviceList(self, req):
    resp = OnvifDeviceListQueryResponse()
    # Statuses of detected devices
    if len(self.detected_onvifs.keys()) == 0:
      resp_status_for_device = OnvifDeviceStatus()
      resp_status_for_device.uuid = "Searching"
      resp_status_for_device.device_name = "Searching"
      resp_status_for_device.manufacturer = ""
      resp_status_for_device.model = ""
      resp_status_for_device.firmware_version = ""
      resp_status_for_device.hardware_id = ""
      resp_status_for_device.host = ""
      resp_status_for_device.port = 0
      resp_status_for_device.connectable = False
      resp_status_for_device.idx_node_running = False
      resp_status_for_device.ptx_node_running = False
      resp.device_statuses.append(resp_status_for_device)
    else:
      for uuid in self.detected_onvifs:
        device = self.detected_onvifs[uuid]
        resp_status_for_device = OnvifDeviceStatus()
        resp_status_for_device.uuid = uuid
        if uuid in self.device_name_dict.keys():
          resp_status_for_device.device_name = self.device_name_dict[uuid]
        else:
          resp_status_for_device.device_name = uuid

        
        # Manufacturer settings
        resp_status_for_device.manufacturer = device['manufacturer']
        resp_status_for_device.model = device['model']
        resp_status_for_device.firmware_version = device['firmware_version']
        resp_status_for_device.hardware_id = device['hardware_id']

        # Network settings
        resp_status_for_device.host = device['host']
        resp_status_for_device.port = device['port']
        resp_status_for_device.connectable = device['connectable']
        resp_status_for_device.idx_node_running = True if (device['idx_subproc'] is not None) else False
        resp_status_for_device.ptx_node_running = True if (device['ptx_subproc'] is not None) else False
        resp.device_statuses.append(resp_status_for_device)
      
      # Known configurations
      for uuid in self.configured_onvifs:
        resp_cfg_for_device = OnvifDeviceCfg()    
        resp_cfg_for_device.uuid = uuid
      
        config = self.configured_onvifs[uuid]
        if uuid in self.device_name_dict.keys():
          resp_cfg_for_device.device_name = self.device_name_dict[uuid]
        else:
          resp_cfg_for_device.device_name = uuid
        resp_cfg_for_device.username = config['username']
        resp_cfg_for_device.password = config['password']
        resp_cfg_for_device.node_base_name = config['node_base_name']
        resp_cfg_for_device.idx_enabled = config['idx_enabled']
        resp_cfg_for_device.ptx_enabled = config['ptx_enabled']
        resp_cfg_for_device.idx_driver = config['idx_driver']
        resp_cfg_for_device.ptx_driver = config['ptx_driver']      
        resp.device_cfgs.append(resp_cfg_for_device)
    
    return resp

  def resyncOnvifDeviceClocks(self, _):
    for uuid in self.detected_onvifs:
      device = self.detected_onvifs[uuid]
      if (device['connectable'] is False) or  (uuid not in self.configured_onvifs):
        continue
      
      config = self.configured_onvifs[uuid]
      basename = config['node_base_name']
      soapSyncSystemDateAndTime(device['host'], device['port'], config['username'], config['password'])

    return EmptyResponse()
  
  def provideDeviceDriverList(self, _):
    resp = OnvifDeviceDriverListQueryResponse()

    idx_drivers_ordered = []
    for driver in self.idx_drivers_dict.keys():
      if driver.find("GENERIC") != -1:
        idx_drivers_ordered.append(driver)
    for driver in self.idx_drivers_dict.keys():
      if driver.find("GENERIC") == -1:
        idx_drivers_ordered.append(driver)
    resp.idx_drivers = idx_drivers_ordered

    ptx_drivers_ordered = []
    for driver in self.ptx_drivers_dict.keys():
      if driver.find("GENERIC") != -1:
         ptx_drivers_ordered.append(driver)
    for driver in self.ptx_drivers_dict.keys():
      if driver.find("GENERIC") == -1:
         ptx_drivers_ordered.append(driver)
    resp.ptx_drivers =  ptx_drivers_ordered
    return resp
  
  def runDiscovery(self, _):
    #nepi_msg.publishMsgInfo(self,'Debug: running discovery')

    # Some devices only respond once to discovery
    clear_disc = nepi_ros.get_param(self,"~clear_discovery",self.init_clear_disc)
    if clear_disc:
      self.wsd.clearRemoteServices()
    detected_services = self.wsd.searchServices(timeout=1)


    #nepi_msg.publishMsgWarn(self,'Detected services ' + str(detected_services))
    detected_uuids = []
    for service in detected_services:
      
      endpoint_ref = service.getEPR()
      #nepi_msg.publishMsgInfo(self,'Detected endpoint' + str(endpoint_ref))
      endpoint_ref_tokens = endpoint_ref.split(':')
      if len(endpoint_ref_tokens) < 3:
        nepi_msg.publishMsgWarn(self,'Detected ill-formed endpoint reference ' + str(endpoint_ref) + ' skipping')
        continue # Ill-formed
      uuid = endpoint_ref_tokens[2]
      # Some devices randomize the first part of their UUID on each reboot, so truncate that off
      if '-' in uuid:
        uuid_tokens = uuid.split('-')[1:]
        uuid = "-".join(uuid_tokens)
      uuid = uuid.upper()
      detected_uuids.append(uuid)
            
      # Query this device and add to our tracked list if not previously done
      if uuid not in self.detected_onvifs:
        xaddr = service.getXAddrs()[0]
        parsed_xaddr = urllib.parse.urlparse(xaddr)
        hostname = parsed_xaddr.hostname
        port = parsed_xaddr.port if parsed_xaddr.port is not None else 80
        
        # Haven't found any universally valid way to determine if a device supports video and/or pan/tilt, so 
        # just hardcoding these as true for now -- up to the user to enable IDX or PTX as applicable
        #is_nvt = False
        #is_ptz = False
        is_nvt = True
        is_ptz = True
        
        for scope in service.getScopes():
          scope_val = scope.getValue()

          if not scope_val.startswith('onvif'):
            # Skip any WSDiscovery device that is not ONVIF
            continue

          # Check for video streaming
          # Commenting out -- see is_nvt note above
          #if scope_val.endswith(self.ONVIF_SCOPE_NVT_ID) or scope_val.endswith(self.ONVIF_SCOPE_NVT_ALT_ID):
          #  is_nvt = True
                  
          # Check for PTZ
          # Commenting out -- see is_ptz note above
          #if scope_val.endswith(self.ONVIF_SCOPE_PTZ_ID):
          #  is_ptz = True

        # Just skip any WSDiscovery device that is not identified as NVT or PTZ
        if (not is_nvt) and (not is_ptz):
          continue


        self.detected_onvifs[uuid] = {
          'manufacturer' : '',
          'model' : '',
          'firmware_version' : '',
          'hardware_id' : '',
          'host': hostname, 
          'port': port, 
          'video': is_nvt, 
          'ptz': is_ptz, 
          'idx_subproc' : None, 
          'idx_node_name': None,
          'ptx_subproc' : None,
          'ptx_node_name': None,
          'connectable' : False,
        }
        # Now determine if it has a config struct
        self.detected_onvifs[uuid]['config'] = self.configured_onvifs[uuid] if uuid in self.configured_onvifs else None
        if self.detected_onvifs[uuid]['config'] is not None:
          # Check if this device can be reached
          connected = self.attemptONVIFConnection(uuid)
          if connected:
            self.detected_onvifs[uuid]['connectable'] = True
            nepi_msg.publishMsgInfo(self,'Connected to device '  + str(uuid) + ' at ' + hostname + ':' + str(port) + ' via configured credentials')

    lost_onvifs = []
    for uuid in self.detected_onvifs:
      detected_onvif = self.detected_onvifs[uuid]
      # Now look for services we've previously detected but are now lost
      lost_connection = False
      #if self.detected_onvifs[uuid]['connectable']:
        #lost_connection = self.attemptONVIFConnection(uuid) == False
      

      if uuid not in detected_uuids or lost_connection:
        nepi_msg.publishMsgWarn(self,'detected uuids: ' + str(detected_uuids)) 
        self.stopAndPurgeNodes(uuid)
        lost_onvifs.append(uuid)
        continue
      

      if detected_onvif['config'] is None:
        continue

      # If not yet connectable, try again here, but don't proceed if still not connectable
      if detected_onvif['connectable'] is False:
        self.detected_onvifs[uuid]['connectable'] = self.attemptONVIFConnection(uuid)
      if not self.detected_onvifs[uuid]['connectable']:
        # Warning logged upstream
        continue

      needs_restart = False

      needs_idx_start = (detected_onvif['video'] is True) and (detected_onvif['idx_subproc'] is None) and \
                        (detected_onvif['config'] is not None) and ('idx_enabled' in detected_onvif['config']) and \
                        (detected_onvif['config']['idx_enabled'] is True)
      if needs_idx_start:
        nepi_msg.publishMsgInfo(self,"IDX node needs start " + str(needs_idx_start) )
      # Check for restarts
      elif (detected_onvif['idx_node_name'] is not None) and (self.nodeIsRunning(detected_onvif['idx_node_name']) is False):
        nepi_msg.publishMsgWarn(self,'IDX node for ' + str(uuid) + ' udrvpectedly not running... will force restart')
        needs_restart = True
      '''
      nepi_msg.publishMsgWarn(self,"*******************************************")
      nepi_msg.publishMsgWarn(self,"IDX entry" + str(uuid) )
      nepi_msg.publishMsgWarn(self,"video: " + str(detected_onvif['video']))
      nepi_msg.publishMsgWarn(self,"subproc: " + str(detected_onvif['idx_subproc']))
      nepi_msg.publishMsgWarn(self,"idx_enabled: " + str((detected_onvif['config']['idx_enabled'] is True)))
      nepi_msg.publishMsgWarn(self,str(detected_onvif['config']))
      '''
      
      needs_ptx_start = (detected_onvif['ptz'] is True) and (detected_onvif['ptx_subproc'] is None) and \
                        (detected_onvif['config'] is not None) and ('ptx_enabled' in detected_onvif['config']) and \
                        (detected_onvif['config']['ptx_enabled'] is True)
      if needs_ptx_start:
        nepi_msg.publishMsgInfo(self,"PTX needs start " + str(needs_ptx_start) )
      # Check for restarts
      elif (detected_onvif['ptx_node_name'] is not None) and (self.nodeIsRunning(detected_onvif['ptx_node_name']) is False):
        nepi_msg.publishMsgWarn(self,'PTX node for ' + str(uuid) + ' udrvpectedly not running... will force restart')
        needs_restart = True
      '''
      nepi_msg.publishMsgWarn(self,"*******************************************")
      nepi_msg.publishMsgWarn(self,"PTX entry" + str(uuid) )
      nepi_msg.publishMsgWarn(self,"video: " + str(detected_onvif['video']))
      nepi_msg.publishMsgWarn(self,"subproc: " + str(detected_onvif['ptx_subproc']))
      nepi_msg.publishMsgWarn(self,"ptx_enabled: " + str((detected_onvif['config']['ptx_enabled'] is True)))
      nepi_msg.publishMsgWarn(self,str(detected_onvif['config']))
      '''
      
      needs_start = needs_idx_start or needs_ptx_start
      # Now ensure that the device is connectable via the known/configured credentials
      if needs_start:
        # Device is connectable, so attempt to start the node(s), 
        self.startNodesForDevice(uuid=uuid, start_idx = needs_idx_start, start_ptx = needs_ptx_start)
      if needs_restart is True:
        self.stopAndPurgeNodes(uuid)
        
    # Finally, purge lost device from our set... can't do it in the detection loop above because it would modify the object 
    # that is being iterated over; throws exception.
    for uuid in lost_onvifs:
      self.detected_onvifs.pop(uuid)

    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_ros.sleep(self.discovery_interval_s,100)
    nepi_ros.timer(nepi_ros.duration(1), self.runDiscovery, oneshot=True)

  def attemptONVIFConnection(self, uuid):
    if uuid not in self.detected_onvifs:
      nepi_msg.publishMsgWarn(self,"Can't attempt ONVIF connection for undetected device... ignoring")
      return False 
    
    config = self.detected_onvifs[uuid]['config']
    if (config is None) or ('username' not in config) or ('password' not in config):
      nepi_msg.publishMsgWarn(self,'Incomplete ONVIF configuration for ' + str(uuid) + ' ... cannot proceed')
      return False
    
    hostname = self.detected_onvifs[uuid]['host']
    port = self.detected_onvifs[uuid]['port']
    host_reachable = (os.system(f"ping -c 1 {hostname}") == 0)
    if host_reachable is False:
      return False
    
    username = config['username']
    password = config['password']

    try:
      dev_info = soapGetDeviceInformation(hostname, str(port), username, password)
      soapSyncSystemDateAndTime(hostname, str(port), username, password)
      #nepi_msg.publishMsgInfo(self,'Connected to device '  + str(uuid) + ' at ' + hostname + ':' + str(port) + ' via configured credentials')
    except Exception as e:
      return False
    
    self.detected_onvifs[uuid]['manufacturer'] = dev_info["Manufacturer"]
    self.detected_onvifs[uuid]['model'] = dev_info["Model"]
    self.detected_onvifs[uuid]['firmware_version'] = dev_info["FirmwareVersion"]
    self.detected_onvifs[uuid]['hardware_id'] = dev_info["HardwareId"]
    self.detected_onvifs[uuid]['serial_num'] = dev_info["SerialNumber"]

    return True    

  def startNodesForDevice(self, uuid, start_idx, start_ptx):
    if uuid not in self.detected_onvifs:
      nepi_msg.publishMsgWarn(self,"Can't start nodes for undetected device... ignoring")
      return False 
    
    config = self.detected_onvifs[uuid]['config']
    if (config is None) or ('node_base_name' not in config) or ('username' not in config) or ('password' not in config):
      nepi_msg.publishMsgWarn(self,'Incomplete configuration for ' + str(uuid) + ' ... cannot proceed')
      return False
      
    serial_num = self.detected_onvifs[uuid]['serial_num'] 
    

    # Should have already ensured all of these exist in attemptONVIFConnection
    username = config['username']
    password = config['password']
    hostname = self.detected_onvifs[uuid]['host']
    port = self.detected_onvifs[uuid]['port']
    
    identifier = hostname.replace(".","")
    device_name = config['node_base_name'] + '_' + identifier
    nepi_msg.publishMsgInfo(self,device_name)
    self.device_name_dict[uuid] = device_name
    if start_idx is True:
      driver_name = self.configured_onvifs[uuid]['idx_driver']
      if driver_name in self.drvs_dict.keys():
        file_name = self.drvs_dict[driver_name]['NODE_DICT']['file_name']
        ros_node_name = config['node_base_name'] + '_camera_' + identifier
        fully_qualified_node_name = self.base_namespace + ros_node_name
        self.checkLoadConfigFile(node_namespace=fully_qualified_node_name)
        drv_dict = self.configured_onvifs[uuid]['idx_drv_dict']
        driver_param_name = ros_node_name + "/drv_dict"
        nepi_ros.set_param(self,driver_param_name,drv_dict)
        self.overrideConnectionParams(fully_qualified_node_name, username, password, hostname, port, config['idx_driver'])
        # And try to launch the node
        nepi_msg.publishMsgInfo(self,'Launching node ' + ros_node_name + ' with file ' + file_name + ' for uuid ' + str(uuid))
        [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, ros_node_name)
        if success == True:
          nepi_msg.publishMsgInfo(self,'Launched driver node ' + ros_node_name )
          self.detected_onvifs[uuid]['idx_subproc'] = sub_process
          self.detected_onvifs[uuid]['idx_node_name'] = ros_node_name
        else:
          nepi_msg.publishMsgWarn(self,'Failed to launch driver node ' + ros_node_name )
      else:
        nepi_msg.publishMsgWarn(self,'Failed to find driver ' + driver_name + ' for launch driver node ' + ros_node_name )

    if start_ptx is True:
      driver_name = self.configured_onvifs[uuid]['ptx_driver']
      if driver_name in self.drvs_dict.keys():
        file_name = self.drvs_dict[driver_name]['NODE_DICT']['file_name']
        ros_node_name = config['node_base_name'] + '_pan_tilt_' + identifier
        fully_qualified_node_name = self.base_namespace + ros_node_name
        self.checkLoadConfigFile(node_namespace=fully_qualified_node_name)
        drv_dict = self.configured_onvifs[uuid]['ptx_drv_dict']
        driver_param_name = ros_node_name + "/drv_dict"
        nepi_ros.set_param(self,driver_param_name,drv_dict)
        self.overrideConnectionParams(fully_qualified_node_name, username, password, hostname, port, config['ptx_driver'])
        # And try to launch the node
        nepi_msg.publishMsgInfo(self,'Launching node ' + ros_node_name + ' with file ' + file_name + ' for uuid ' + str(uuid))
        [success, msg, sub_process] = nepi_drv.launchDriverNode(file_name, ros_node_name)
        if success == True:
          nepi_msg.publishMsgInfo(self,'Launched driver node ' + ros_node_name )
          self.detected_onvifs[uuid]['ptx_subproc'] = sub_process
          self.detected_onvifs[uuid]['ptx_node_name'] = ros_node_name
        else:
          nepi_msg.publishMsgWarn(self,'Failed to launch driver node ' + ros_node_name )
      else:
        nepi_msg.publishMsgWarn(self,'Failed to find driver ' + driver_name + ' for launch driver node ' + ros_node_name )

  def overrideConnectionParams(self, fully_qualified_node_name, username, password, hostname, port, driver_id):
    credentials_ns = fully_qualified_node_name + '/credentials/'
    network_ns = fully_qualified_node_name + '/network/'
    nepi_ros.set_param(self,credentials_ns + 'username', username)
    nepi_ros.set_param(self,credentials_ns + 'password', password)
    nepi_ros.set_param(self,network_ns + 'host', hostname)
    nepi_ros.set_param(self,network_ns + 'port', port)
    nepi_ros.set_param(self,fully_qualified_node_name + '/driver_id', str(driver_id))
  
    
  def stopAndPurgeNodes(self, uuid):
    device = self.detected_onvifs[uuid]
    subprocs = [device['idx_subproc'], device['ptx_subproc']]
                 
    for p in subprocs:
      if (p is not None) and (p.poll() is None):
        p.terminate()
        terminate_timeout = 3
        node_dead = False
        while (terminate_timeout > 0):
          time.sleep(1)
          if (p.poll() is None):
            terminate_timeout -= 1
          else:
            node_dead = True
            break
        if not node_dead:
          p.kill()
          time.sleep(1)
        
    self.detected_onvifs[uuid]['idx_subproc'] = None
    self.detected_onvifs[uuid]['ptx_subproc'] = None       

  def subprocessIsRunning(self, subproc):
    if subproc.poll() is not None:
      return True
    return False
  
  def nodeIsRunning(self, node_name):
    running = False
    node_list = nepi_ros.get_node_list()
    for node in node_list:
      if node.find(node_name) != -1:
        running = True
    if running == False:
      nepi_msg.publishMsgWarn(self,"Failed to find node_name: " + str(node_name) + " in node list: " + str(node_list))
    return running
  
  def checkLoadConfigFile(self, node_namespace):
    ros_node_name = node_namespace.split('/')[-1]
    folder_name = "drivers/" + ros_node_name
    node_config_file_folder = os.path.join(self.DEFAULT_NEPI_CONFIG_PATH, folder_name)
    nepi_msg.publishMsgInfo(self,node_config_file_folder)
    # Just make the folder if necessary 
    os.makedirs(node_config_file_folder, exist_ok=True)

    full_path_config_file = os.path.join(node_config_file_folder, ros_node_name + ".yaml")
    full_path_config_file_factory = full_path_config_file + ".num_factory"
    if not os.path.exists(full_path_config_file_factory):
      nepi_msg.publishMsgWarn(self,'No existing config. infrastructure for ' + ros_node_name + ' ... creating folders and empty file')
      with open(full_path_config_file_factory, 'w') as f:
        f.write('# This factory config file was auto-generated by NEPI onvif manager')
      os.symlink(full_path_config_file_factory, full_path_config_file)
    
    nepi_msg.publishMsgInfo(self,self.node_name + ": Loading parameters from " + full_path_config_file + " for " + node_namespace)
    rosparam_load_cmd = ['rosparam', 'load', full_path_config_file, node_namespace]
    subprocess.run(rosparam_load_cmd)
        
    # This doesn't work -- returns a dictionary, but you must still use rosparam.upload_params() to get them to the server!!!
    #rosparam.load_file(filename = full_path_config_file, default_namespace = node_namespace)

  def setCurrentSettingsAsDefault(self):
    nepi_ros.set_param(self,'~discovery_interval_s', self.discovery_interval_s)
    nepi_ros.set_param(self,'autosave_cfg_changes', self.autosave_cfg_changes)
    nepi_ros.set_param(self,'~onvif_devices', self.configured_onvifs)
  
  def updateFromParamServer(self):
    self.discovery_interval_s = nepi_ros.get_param(self,'~discovery_interval_s', self.discovery_interval_s)
    self.autosave_cfg_changes = nepi_ros.get_param(self,'~autosave_cfg_changes', self.autosave_cfg_changes)
    self.configured_onvifs = nepi_ros.get_param(self,'~onvif_devices', self.configured_onvifs)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")

# Direct SOAP calls
DEVICE_SERVICE_PATH = "/onvif/device_service"

def soapGetDeviceInformation(globalip, globalport, username, password):
    soap_env = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:t="http://www.onvif.org/ver10/device/wsdl">
            <s:Header>
                <Security s:mustUnderstand="false" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
                <t:GetDeviceInformation/>
            </s:Body>
        </s:Envelope>
        """
    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    # Send HTTP request
    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    soap_resp = requests.post(url, data=soap_env, headers=headers, timeout=30)


    # Initialize to unknown
    device_info = {
        "Manufacturer" : "Unknown",
        "Model" : "Unknown",
        "FirmwareVersion" : "Unknown",
        "SerialNumber" : "Unknown",
        "HardwareId" : "Unknown"
    }

    if soap_resp.status_code != 200:
      nepi_msg.publishMsgWarn(self,'Got error response from onvif device at ' + globalip + ' when querying device info')
      return device_info
    
    root = ET.fromstring(soap_resp.content)

    namespaces = {
        'tds': 'http://www.onvif.org/ver10/device/wsdl',
    }

    dev_info_xml = root.find('.//tds:GetDeviceInformationResponse', namespaces=namespaces)
    device_info["Manufacturer"] = dev_info_xml.find('.//tds:Manufacturer', namespaces=namespaces).text
    device_info["Model"] = dev_info_xml.find('.//tds:Model', namespaces=namespaces).text
    device_info["FirmwareVersion"] = dev_info_xml.find('.//tds:FirmwareVersion', namespaces=namespaces).text
    device_info["SerialNumber"] = dev_info_xml.find('.//tds:SerialNumber', namespaces=namespaces).text
    device_info["HardwareId"] = dev_info_xml.find('.//tds:HardwareId', namespaces=namespaces).text

    return device_info

def soapSyncSystemDateAndTime(globalip, globalport, username, password):
    now = datetime.datetime.now(timezone.utc)
    soap_env = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:tds="http://www.onvif.org/ver10/device/wsdl"
                    xmlns:tt="http://www.onvif.org/ver10/schema">
            <s:Header>
                <Security s:mustUnderstand="false" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
              <tds:SetSystemDateAndTime>
                <tds:DateTimeType>Manual</tds:DateTimeType>
                <tds:DaylightSavings>false</tds:DaylightSavings>
                <tds:UTCDateTime>
                  <tt:Time>
                    <tt:Hour>{now.hour}</tt:Hour>
                    <tt:Minute>{now.minute}</tt:Minute>
                    <tt:Second>{now.second}</tt:Second>
                  </tt:Time>
                  <tt:Date>
                    <tt:Year>{now.year}</tt:Year>
                    <tt:Month>{now.month}</tt:Month>
                    <tt:Day>{now.day}</tt:Day>
                  </tt:Date>
                </tds:UTCDateTime>
              </tds:SetSystemDateAndTime>
            </s:Body>
        </s:Envelope>
        """
    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    #nepi_msg.publishMsgWarn(self,'Debug: Sending SetSystemTimeAndDate soap_env ' + str(soap_env))
    # Send HTTP request
    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    soap_resp = requests.post(url, data=soap_env, headers=headers, timeout=30)
    #nepi_msg.publishMsgWarn(self,Debug: Got back ' + str(soap_resp.text))



if __name__ == '__main__':
  node = ONVIFMgr()



