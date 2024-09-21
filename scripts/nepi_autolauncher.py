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
import glob
import subprocess
import time
import rospy

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

# Specialized discovery functions
from nepi_drivers_rbx.mavlink_rbx_auto_discovery import mavlink_discover
from nepi_drivers_lsx.sealite_auto_discovery import sealite_discover

class NEPIAutolauncher:
  DEFAULT_NODE_NAME = "nepi_autolauncher"

  NEPI_DEFAULT_CFG_PATH = '/opt/nepi/ros/etc/'
  DEVICE_CHECK_INTERVAL_S = 3.0

  # Path devices are those that are discovered/undiscovered entirely based on existence of a filesystem
  # path. They are fully handled by this class and can be defined through configuration (in contrast with
  # specialized devices (e.g., mavlink, sealite) that do not generate unique filesystem paths when connected,
  # hence must be discovered via additional protocol-level logic)
  DEFAULT_PATH_DEVICE_LIST = [
    {
      'path': '/dev/iqr_pan_tilt',
      'ros_package': 'nepi_drivers_ptx',
      'ros_node_type': 'iqr_ros_pan_tilt_node',
      'ros_node_basename': 'iqr_pan_tilt',
      'ros_params': [],
      'config_file_list': ['dummy_config']
    },
  ]

  def __init__(self):
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    self.node_name = rospy.get_name().split('/')[-1]

    self.launchedPathDevices = []
    self.active_path_list = []






    self.resetFromParamServer() # Set up parameters
    
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.updateParamServer, paramsModifiedCallback=self.resetFromParamServer)

    rospy.Timer(rospy.Duration(self.DEVICE_CHECK_INTERVAL_S), self.detectAndManageDevices)
    rospy.spin()

  def updateParamServer(self):
    rospy.set_param('~path_device_list', self.path_detections)

  def resetFromParamServer(self):
    self.path_detections = rospy.get_param('~path_device_list', self.DEFAULT_PATH_DEVICE_LIST)

  def detectAndManagePathDevices(self):
    # First, clear the still_present flags so that we can reidentify and kill any that aren't running
    for launched in self.launchedPathDevices:
      #rospy.loginfo('Debug: Know about ' + str(launched))
      launched['still_present'] = False

    # Look for all the known plug-and-play filesystem paths
    for potential in self.path_detections:
      potential_path = potential['path']

      path_match_list = glob.glob(potential_path)
      for i,path in enumerate(path_match_list):
        # Check if this path is already handled
        already_launched = False
        for launched in self.launchedPathDevices:
          if launched['paths'][0] == path:
            launched['still_present'] = True 
            already_launched = True   
        
        if already_launched == True:
          continue
             
        # If we get here, this path has not yet been handled
        ros_package = potential['ros_package']
        ros_node_type = potential['ros_node_type']
        ros_node_basename = ros_node_type
        if 'ros_node_basename' in potential:
          ros_node_basename = potential['ros_node_basename']

        specifier = "_" + str(i) if (len(path_match_list) > 1) else ""
        ros_node_name = ros_node_basename + specifier
          
        self.checkLoadConfigFile(node_name=ros_node_name)
        
        # And try to launch the node
        ros_params = [] if 'ros_params' not in potential else potential['ros_params']
        self.startNode(package=ros_package, node_type=ros_node_type, node_name=ros_node_name, dev_path=path, params=ros_params)

    # Now kill off any previously started nodes whose detection has failed
    for i,launched in enumerate(self.launchedPathDevices):
      if launched['still_present'] is False:
        self.stopAndPurgeNode(node_index=i)
    
  def detectAndManageDevices(self, _): # Extra arg since this is a rospy Timer callback
    # First, check over all active paths and make sure they still exist. If not, remove them.
    # This should be used as a signal that hardware is detached in the custom detectors
    for i,path in enumerate(self.active_path_list):
      if not os.path.exists(path):
        del self.active_path_list[i]
    
    # "Path devices" are fully handled within this class
    self.detectAndManagePathDevices()

    # Others leverage custom detectors -- these detectors must take in the active_path_list to avoid
    # manipulating those paths (e.g., serial port settings) and then return a modified list that includes
    # any paths that they are leveraging so that others won't try to manipulate those
    try:
      self.active_path_list = mavlink_discover(self.active_path_list)
    except Exception as e:
      rospy.logwarn("%s: Mavlink discovery threw an exception (%s)", self.node_name, str(e))

    try:
      self.active_path_list = sealite_discover(self.active_path_list)
    except Exception as e:
      pass
      #rospy.logwarn("%s: Sealite discovery threw an exception (%s)", self.node_name, str(e))

  def startNode(self, package, node_type, node_name, dev_path, params=[]):
    node_namespace = rospy.get_namespace() + node_name
    rospy.loginfo(self.node_name + ": Starting new node " + node_namespace + " (" + package + ":" + node_type + ")")

    # Now start the node via rosrun
    node_run_cmd = ['rosrun', package, node_type, '__name:=' + node_name, '_device_path:=' + dev_path] + params
    p = subprocess.Popen(node_run_cmd)
    if p.poll() is not None:
      rospy.logerr("Failed to start " + node_name + " via " + " ".join(x for x in node_run_cmd) + " (rc =" + str(p.returncode) + ")")
    else:
      dev_paths = [dev_path]
      # Check if the dev_path is a symlink, in which case we must mark the link target(s) as active as well
      # Walk the entire symlink path
      path_str = dev_path
      while os.path.islink(path_str):
        new_path_str = os.readlink(path_str)
        if new_path_str[0] != '/': # Not an absolute path
          new_path_str = os.path.join(os.path.dirname(path_str), new_path_str) # Force absolute
        #rospy.logwarn('Debug: %s-->%s', path_str, new_path_str)
        path_str = new_path_str
        dev_paths.append(path_str)
      
      self.active_path_list += dev_paths # Append the full list
            
      self.launchedPathDevices.append({'node_name': node_name, 
                                   'node_subprocess': p, 
                                   'still_present': True,
                                   'paths': dev_paths})
  
  def stopAndPurgeNode(self, node_index):
    if node_index >= len(self.launchedPathDevices):
      rospy.logwarn(self.node_name + ": Cannot stop node with out-of-bounds index " + str(node_index))
      return
    
    node_dict = self.launchedPathDevices[node_index]
    rospy.loginfo(self.node_name + ": stopping " + node_dict['node_name'])
    if node_dict['node_subprocess'].poll() is None:
      node_dict['node_subprocess'].terminate()
      terminate_timeout = 3
      node_dead = False
      while (terminate_timeout > 0):
        rospy.loginfo(self.node_name + ": Attempting to terminating " + node_dict['node_name'] + " gracefully")
        time.sleep(1)
        if node_dict['node_subprocess'].poll() is None:
          terminate_timeout -= 1
        else:
          node_dead = True
          break
        
      if not node_dead:
        rospy.logwarn(self.node_name + ": Failed to terminate " + node_dict['node_name'] + " gracefully... attempting to kill")
        # Escalate it
        node_dict['node_subprocess'].kill()
        time.sleep(1)
    else:
      rospy.logwarn(self.node_name + ": " + node_dict['node_name'] + " is not running")

    # Turns out that is not always enough to get the node out of the ros system, so we use rosnode cleanup, too
    # rosnode cleanup won't find the disconnected node until the process is fully terminated
    try:
      node_dict['node_subprocess'].wait(timeout=10)
    except:
      pass

    #rospy.sleep(10) # Long enough for process to die and rosnode cleanup to see the node as disconnected
    cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
    try:
      cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
      cleanup_proc.wait(timeout=10) 
    except Exception as e:
      rospy.logwarn('%s: rosnode cleanup of %s failed (%s)', self.node_name, node_dict['node_name'], str(e))    
        
    # And remove it from the member variables
    for path in node_dict['paths']:
      if path in self.active_path_list:
        self.active_path_list.remove(path)
    
    self.launchedPathDevices.pop(node_index)
     
    return
  
  def nodeIsRunning(self, node_name):
    for launched in self.launchedPathDevices:
      if launched['node_name'] == node_name:
        if launched['node_subprocess'].poll() is not None:
          return False
        else:
          return True
    # If we get here, didn't find the node in our list    
    rospy.logwarn(self.node_name + ": cannot check run status of unknown node " + node_name)
    return False
  
  def checkLoadConfigFile(self, node_name):
    config_folder = os.path.join(self.NEPI_DEFAULT_CFG_PATH, node_name)
    if not os.path.isdir(config_folder):
      rospy.logwarn(self.node_name + ': No config folder found for %s... creating one at %s', node_name, config_folder)
      os.makedirs(name = config_folder, mode = 0o775)
      return
    
    config_file = os.path.join(config_folder, node_name + ".yaml")
    node_namespace = rospy.get_namespace() + node_name
    if os.path.exists(config_file):
      rospy.loginfo(self.node_name + ": Loading parameters from " + config_file + " to " + node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_namespace)
      #rosparam.load_file(filename = config_file, default_namespace = node_name)
      # Seems programmatic rosparam.load_file is not working at all, so use the command-line version instead
      rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
      subprocess.run(rosparam_load_cmd)
    else:
      rospy.logwarn(self.node_name + ": No config file found for " + node_name + " in " + self.NEPI_DEFAULT_CFG_PATH)
    
if __name__ == '__main__':
  node = NEPIAutolauncher()            

        
      

 
