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
import subprocess
import threading
import traceback
import yaml
#import resource
import time
import psutil

import rospy

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg 

from std_msgs.msg import Empty
from nepi_ros_interfaces.srv import (
    GetScriptsQuery,
    GetScriptsQueryResponse,
    GetRunningScriptsQuery,
    GetRunningScriptsQueryResponse,
    LaunchScript,
    LaunchScriptRequest,
    LaunchScriptResponse,
    StopScript,
    GetSystemStatsQuery,
    GetSystemStatsQueryResponse,
    SystemStorageFolderQuery
)

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import AutoStartEnabled


class AutomationManager:
    AUTOMATION_DIR = "/mnt/nepi_storage/automation_scripts"
    DEFAULT_SCRIPT_STOP_TIMEOUT_S = 10.0
    SCRIPT_LOG_PATH = "/mnt/nepi_storage/logs/automation_script_logs"

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "automation_manager" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################
        
        get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
        nepi_msg.publishMsgInfo(self,"Waiting for system automation scripts folder query service " + get_folder_name_service)
        rospy.wait_for_service(get_folder_name_service)
        nepi_msg.publishMsgInfo(self,"Calling system automation scripts folder query service " + get_folder_name_service)
        try:
            nepi_msg.publishMsgInfo(self,"Getting automation scripts folder query service " + get_folder_name_service)
            folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)

            response = folder_query_service('automation_scripts')
            nepi_msg.publishMsgInfo(self,"Got automation scripts folder path" + response.folder_path)
            self.AUTOMATION_DIR = response.folder_path
            
            response = folder_query_service('logs/automation_script_logs')
            nepi_msg.publishMsgInfo(self,"Got automation scripts log folder path" + response.folder_path)
            self.SCRIPT_LOG_PATH = response.folder_path
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain automation scripts folder, falling back to: " + self.AUTOMATION_DIR + " " + str(e))


        self.script_stop_timeout_s = nepi_ros.get_param(self,'~script_stop_timeout_s', self.DEFAULT_SCRIPT_STOP_TIMEOUT_S)

        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        self.scripts = self.get_scripts()
        self.file_sizes = self.get_file_sizes()

        self.processes = {}

        self.script_counters = {}
        for script in self.scripts:
            #TODO: These should be gathered from a stats file on disk to remain cumulative for all time (clearable on ROS command)
            self.script_counters[script] = {'started': 0, 'completed': 0, 'stopped_manually': 0, 'errored_out': 0, 'cumulative_run_time': 0.0}

        self.script_configs = {} # Dictionary of dictionaries  
        self.setupScriptConfigs()
        self.updateFromParamServer() # Call this to grab the parameters initially
                
        self.running_scripts = set()

        self.get_scripts_service = rospy.Service("get_scripts", GetScriptsQuery, self.handle_get_scripts)
        self.get_running_scripts_service = rospy.Service("get_running_scripts", GetRunningScriptsQuery, self.handle_get_running_scripts)
        self.launch_script_service = rospy.Service("launch_script", LaunchScript, self.handle_launch_script)
        self.stop_script_service = rospy.Service("stop_script", StopScript, self.handle_stop_script)
        self.get_system_stats_service = rospy.Service("get_system_stats", GetSystemStatsQuery, self.handle_get_system_stats)

        self.monitor_thread = threading.Thread(target=self.monitor_scripts)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        self.watch_thread = threading.Thread(target=self.watch_directory, args=(self.AUTOMATION_DIR, self.on_file_change))
        self.watch_thread.daemon = True
        self.watch_thread.start()

        # Subscribe to topics
        rospy.Subscriber('enable_script_autostart', AutoStartEnabled, self.AutoStartEnabled_cb)

        # Autolaunch any scripts that are so-configured
        for script_name in self.script_configs:
            script_config = self.script_configs[script_name]
            if script_config['auto_start'] is True:
                nepi_msg.publishMsgInfo(self,"Auto-starting " + script_name)
                req = LaunchScriptRequest(script_name)
                self.handle_launch_script(req)

        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################

    def setupScriptConfigs(self, single_script=None):
        script_list = []
        if single_script is None:
            script_list = self.scripts
        else:
            script_list = [single_script]

        # Ensure all known scripts have a script_config
        for script_name in script_list:
            if script_name not in self.script_configs:
                #nepi_msg.publishMsgWarn(self,"Initializing config for " + script_name)
                self.script_configs[script_name] = {'auto_start': False, 'cmd_line_args': ''}

                # Good place to dos2unix it
                subprocess.call(['dos2unix', os.path.join(self.AUTOMATION_DIR, script_name)])

        # And make sure any unknown scripts don't
        configs_to_delete = [] # Can't delete configs while iterating over them -- runtime error, so just capture them here and then delete in a fresh loop
        for script_name in self.script_configs:
            if script_name not in self.scripts:
                nepi_msg.publishMsgInfo(self,"Purging unknown script " + script_name + "from script_configs")
                configs_to_delete.append(script_name)
        for script_name in configs_to_delete:
            del self.script_configs[script_name]

    def update_script_configs(self):
        #nepi_msg.publishMsgWarn(self,"Ready to run update_script_configs!!!")
        script_configs = {} # Dictionary of dictionaries
        try:
            script_configs = nepi_ros.get_param(self,'~script_configs')
        except KeyError:
            #nepi_msg.publishMsgWarn(self,"Parameter ~script_configs does not exist")
            script_configs = {}

        for script_name in script_configs:
            if script_name not in self.scripts:
                nepi_msg.publishMsgWarn(self,"Config file includes configuration for unknown script " + script_name  + "... skipping this config")
                continue

            script_config = script_configs[script_name]
            nepi_msg.publishMsgInfo(self,script_name + " configuration from param server: " + str(script_config))

            if 'auto_start' not in script_config or 'cmd_line_args' not in script_config:
                nepi_msg.publishMsgWarn(self,"Invalid config. file settings for script " + script_name + "... skipping this config")
                continue

            self.script_configs[script_name]['auto_start'] = script_config['auto_start']
            self.script_configs[script_name]['cmd_line_args'] = script_config['cmd_line_args']
        
    def AutoStartEnabled_cb(self, msg):
        if msg.script not in self.script_configs:
            nepi_msg.publishMsgWarn(self,"Cannot configure autostart for unknown script " + msg.script)
            return
        
        nepi_msg.publishMsgInfo(self,"Script AUTOSTART : " + msg.script + " - " + str(msg.enabled))
        
        self.script_configs[msg.script]['auto_start'] = msg.enabled

        # This is an unusual parameter in that it triggers an automatic save of the config file
        # so update the param server, then tell it to save the file via store_params
        # saveConfig() will trigger the setCurrentSettingsAsDefault callback, so param server will
        # be up-to-date before the file gets saved
        self.save_cfg_if.saveConfig(Empty)

    def updateFromParamServer(self):
        # Read the script_configs parameter from the ROS parameter server
        self.update_script_configs()

    def setCurrentSettingsAsDefault(self):
        nepi_ros.set_param(self,'~script_configs', self.script_configs)
        nepi_ros.set_param(self,'~script_stop_timeout_s', self.script_stop_timeout_s)
        
    def get_scripts(self):
        """
        Detect and report automation scripts that exist in a particular directory in the filesystem.
        """
        #self.scripts = self.get_scripts()
        scripts = []
        for filename in os.listdir(self.AUTOMATION_DIR):
            filepath = os.path.join(self.AUTOMATION_DIR, filename)
            if not os.path.isdir(filepath):
                scripts.append(filename)
        return scripts

    def watch_directory(self, directory, callback):
        files_mtime = {}

        while True:
            for file in os.listdir(directory):
                file_path = os.path.join(directory, file)
                if os.path.isfile(file_path):
                    current_mtime = os.path.getmtime(file_path)
                    if file not in files_mtime or files_mtime[file] != current_mtime:
                        files_mtime[file] = current_mtime
                        callback(file_path, file_deleted=False)
            # And check for deleted files, too
            deleted_files = []
            for file in files_mtime.keys():
                if file not in os.listdir(directory):
                    deleted_files.append(file)
                    callback(os.path.join(directory, file), file_deleted=True)
            for file in deleted_files:
                del files_mtime[file]
            time.sleep(1)

    def on_file_change(self, file_path, file_deleted):
        script_name = os.path.basename(file_path)
        #nepi_msg.publishMsgInfo(self,"File change detected: %s", os.path.basename(script_name))
        
        #Update the scripts list
        self.scripts = self.get_scripts()
        if file_deleted is False:
            # Update the script config here to set up the new/modified script
            self.setupScriptConfigs(single_script=script_name)

            # Update the file size in case it changed
            if os.path.exists(file_path):
                try:
                    f_size = os.path.getsize(file_path)
                except:
                    f_size = 0
            else:
                f_size = 0
            self.file_sizes[script_name] = f_size

            # Reset the script counters entry for this file... consider this a new script
            self.script_counters[script_name] = {'started': 0, 'completed': 0, 'stopped_manually': 0, 'errored_out': 0, 'cumulative_run_time': 0.0}
        else:
            # Just delete this script from all relevant dictionaries
            self.script_configs.pop(script_name, None)
            self.file_sizes.pop(script_name, None)
            self.script_counters.pop(script_name, None)

    def get_file_sizes(self):
        """
        Get the file sizes of the automation scripts in the specified directory.
        """
        file_sizes = {}
        for filename in self.scripts:
            filepath = os.path.join(self.AUTOMATION_DIR, filename)
            file_size = os.path.getsize(filepath)
            file_sizes[filename] = file_size
        return file_sizes

    def handle_get_scripts(self, req):
        return GetScriptsQueryResponse(sorted(self.scripts))
    
    def handle_get_running_scripts(self, req):
        """
        Handle a request to get a list of currently running scripts.
        """
        running_scripts = sorted(list(self.running_scripts))
        #nepi_msg.publishMsgInfo(self,"Running scripts: %s" % running_scripts)

        return GetRunningScriptsQueryResponse(running_scripts)

    def handle_launch_script(self, req):
        """
        Handle a request to launch an automation script.
        """            
        if req.script in self.scripts:
            if req.script not in self.running_scripts:
                try:
                    # Ensure the script is executable (and readable/writable -- why not)
                    script_full_path = os.path.join(self.AUTOMATION_DIR, req.script)
                    os.chmod(script_full_path, 0o774)

                    # Set up logfile. Because we pipe stdout from the script into the file, we must pay attention to buffering at
                    # multiple levels. open() and Popen() are set for line buffering, and we launch the script in a PYTHONUNBUFFERED environment
                    # to ensure that the script print() calls don't get buffered... without that last one, everything is buffered at 8KB no matter
                    # what open() and Popen() are set to.
                    process_cmdline = [script_full_path] + self.script_configs[req.script]['cmd_line_args'].split()
                    script_logfilename = os.path.join(self.SCRIPT_LOG_PATH, req.script + '.log')
                    script_logfile = open(script_logfilename, 'wt', buffering=1) # buffering=1 ==> Line buffering
                    curr_env = os.environ.copy()
                    curr_env['PYTHONUNBUFFERED'] = 'on'
                    
                    process = subprocess.Popen(process_cmdline, stdout=script_logfile, stderr=subprocess.STDOUT, bufsize=1, env=curr_env) # bufsize=1 ==> Line buffering
                    self.processes[req.script] = {'process': process, 'pid': process.pid, 'start_time': psutil.Process(process.pid).create_time(), 'logfile': script_logfile}
                    self.running_scripts.add(req.script)  # Update the running_scripts set
                    nepi_msg.publishMsgInfo(self,"%s: running" % req.script)
                    self.script_counters[req.script]['started'] += 1  # update the counter
                    return LaunchScriptResponse(True)
                except Exception as e:
                    return LaunchScriptResponse(False)
            else:
                nepi_msg.publishMsgWarn(self,"is already running... will not start another instance " +  str(req.script))
                return LaunchScriptResponse(False)
        else:
            nepi_msg.publishMsgInfo(self,"%s: not found" % req.script)
            return LaunchScriptResponse(False)

    def handle_stop_script(self, req):
        """
        Handle a request to stop an automation script.
        """
        if req.script in self.processes:
            process = self.processes[req.script]['process']
            retval = False
            try:
                process.terminate()
                process.wait(timeout=self.script_stop_timeout_s)
                self.script_counters[req.script]['cumulative_run_time'] += (nepi_ros.time_now() - nepi_ros.time_from_sec(self.processes[req.script]['start_time'])).to_sec()
                self.processes[req.script]['logfile'].close()
                del self.processes[req.script]
                self.running_scripts.remove(req.script)  # Update the running_scripts set
                nepi_msg.publishMsgInfo(self,"%s: stopped" % req.script)
                self.script_counters[req.script]['stopped_manually'] += 1  # update the counter
                retval = True
            except Exception as e:
                process.kill()
                try:
                    process.wait(timeout=self.script_stop_timeout_s)
                    retval = True
                except Exception as e2:
                    nepi_msg.publishMsgWarn(self,"Failed to kill process" + str(e2))
                    retval = False
            
            return retval
        else:
            nepi_msg.publishMsgWarn(self,"Script not running " + str(req.script))
            return False
    
    def monitor_scripts(self):
        """
        Monitor the status of all automation scripts.
        """
        while not nepi_ros.is_shutdown():
            for script in self.scripts:
                if script in self.processes:
                    process = self.processes[script]
                    if process['process'].poll() is not None:
                        del self.processes[script]
                        self.running_scripts.remove(script)  # Update the running_scripts set
                                                
                        process['logfile'].close()
                        if process['process'].returncode == 0:
                            self.script_counters[script]['completed'] += 1
                            nepi_msg.publishMsgInfo(self,"%s: completed" % script)
                        else:
                            self.script_counters[script]['errored_out'] += 1
                                               
                        # Update the cumulative run time whether exited on success or error
                        self.script_counters[script]['cumulative_run_time'] += (nepi_ros.time_now() - nepi_ros.time_from_sec(process['start_time'])).to_sec()
                        process['logfile'].close()
        
            # Output script counters
            for script, counter in self.script_counters.items():
              nepi_ros.sleep(1)

    def handle_get_system_stats(self, req):
        script_name = req.script
        response = GetSystemStatsQueryResponse(cpu_percent=None, memory_percent=None, run_time_s=None,
                                               cumulative_run_time_s=None, file_size_bytes=None, log_size_bytes=None, started_runs=None,
                                               completed_runs=None, error_runs=None, stopped_manually=None, auto_start_enabled=None)

        if (script_name not in self.file_sizes) or (script_name not in self.script_counters) or (script_name not in self.script_configs):
            nepi_msg.printMsgThrottle(10, "Requested script not found: %s" % script_name)
            return response # Blank response

        # Get file size for the script_name
        response.file_size_bytes = self.file_sizes[script_name]

        # Get the counter values for the script_name
        response.started_runs = self.script_counters[script_name]['started']
        response.completed_runs = self.script_counters[script_name]['completed']
        response.error_runs = self.script_counters[script_name]['errored_out']
        response.stopped_manually = self.script_counters[script_name]['stopped_manually']
        response.cumulative_run_time_s = self.script_counters[script_name]['cumulative_run_time']
        
        # And config info we want to feed back... TODO: maybe these should be part of a totally separate request
        response.auto_start_enabled = self.script_configs[script_name]['auto_start']

        # Check if the script_name is in the running list
        if (script_name not in self.running_scripts):
            try:
                response.log_size_bytes = os.path.getsize(os.path.join(self.SCRIPT_LOG_PATH, script_name + ".log"))
            except:
                pass
            return response  # Only includes the 'static' info
        
        pid = self.processes[script_name]['pid']
        response.log_size_bytes = os.fstat(self.processes[script_name]['logfile'].fileno()).st_size
        #nepi_msg.publishMsgInfo(self,"PID for script %s: %d" % (script_name, pid))

        try:
            # Get resource usage for the specific PID
            #usage = resource.getrusage(resource.RUSAGE_CHILDREN)
            process = psutil.Process(pid)

            # Get CPU usage
            #self.cpu_percent = (usage.ru_utime + usage.ru_stime) / os.sysconf("SC_CLK_TCK")
            response.cpu_percent = process.cpu_percent(0.1)

            # Get memory usage
            #self.memory_usage = usage.ru_maxrss
            response.memory_percent = 100.0 * float(process.memory_full_info().uss) / float(psutil.virtual_memory().total)
                        
            # Get creation/start-up time
            response.run_time_s = (nepi_ros.time_now() - nepi_ros.time_from_sec(process.create_time())).to_sec()
            # The script_counters cumulative run time only gets updated on script termination, so to keep this value moving in the response,
            # increment it here.
            response.cumulative_run_time_s += response.run_time_s
            #nepi_msg.publishMsgInfo(self,"CPU Percent: %.5f%%, Memory Usage: %.5f%%, Run Time: %.2f" % (response.cpu_percent, response.memory_percent, response.run_time_s))

        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Error gathering running stats: " + str(e))
            return response  # Add new None values for the counters
        
        # Return the system stats as a GetSystemStatsQuery response object
        return response


if __name__ == '__main__':
    AutomationManager()
