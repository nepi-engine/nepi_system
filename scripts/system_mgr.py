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
import shutil
from collections import deque
import re
from datetime import datetime

import subprocess

import rospy

from std_msgs.msg import String, Empty, Float32
from nepi_ros_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveData
from nepi_ros_interfaces.srv import SystemDefsQuery, SystemDefsQueryResponse, OpEnvironmentQuery, OpEnvironmentQueryResponse, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryResponse, SystemStorageFolderQuery, SystemStorageFolderQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
import nepi_edge_sdk_base.nepi_software_update_utils as sw_update_utils

BYTES_PER_MEGABYTE = 2**20


class SystemMgrNode():
    NODE_NAME = "system_mgr"

    STATUS_PERIOD = 1.0  # TODO: Configurable update period?

    DISK_FULL_MARGIN_MB = 250  # MB TODO: Configurable?

    SYS_ENV_PATH = "/opt/nepi/sys_env.bash"
    FW_VERSION_PATH = "/opt/nepi/ros/etc/fw_version.txt"

    status_msg = SystemStatus()
    system_defs_msg = SystemDefs()

    storage_mountpoint = "/mnt/nepi_storage"
    data_folder = storage_mountpoint + "/data"

    storage_uid = 1000 # default to nepi
    storage_gid = 130 # default to "sambashare" # TODO This is very fragile

    REQD_STORAGE_SUBDIRS = ["ai_models", 
                            "automation_scripts", 
                            "data", 
                            "databases", 
                            "license", 
                            "logs", 
                            "logs/automation_script_logs", 
                            "nepi_full_img", 
                            "nepi_full_img_archive", 
                            "nepi_src",
                            "nepi_src/nepi_drivers", 
                            "user_cfg",
                            "sample_data"]
    
    DRIVERS_PATH = '/opt/nepi/ros/lib/nepi_drivers'

    # disk_usage_deque = deque(maxlen=10)
    # Shorter period for more responsive updates
    disk_usage_deque = deque(maxlen=3)

    first_stage_rootfs_device = "/dev/mmcblk0p1"
    nepi_storage_device = "/dev/nvme0n1p3"
    new_img_staging_device = "/dev/nvme0n1p3"
    new_img_staging_device_removable = False
    usb_device = "/dev/sda" 
    sd_card_device = "/dev/mmcblk1p"
    emmc_device = "/dev/mmcblk0p"
    ssd_device = "/dev/nvme0n1p"
    auto_switch_rootfs_on_new_img_install = True
    sw_update_progress = ""

    installing_new_image = False
    archiving_inactive_image = False

    def add_info_string(self, string, level):
        self.status_msg.info_strings.append(StampedString(
            timestamp=rospy.get_rostime(), payload=string, priority=level))

    def get_device_sn(self):
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_SN="):
                    return line.split('=')[1].rstrip()
        return "undefined"

    def get_fw_rev(self):
        if (os.path.exists(self.FW_VERSION_PATH) and (os.path.getsize(self.FW_VERSION_PATH) > 0)):
            with open(self.FW_VERSION_PATH, "r") as f:
                return f.readline().strip()
        return "UNSPECIFIED"
    
    def get_device_type(self):
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_TYPE="):
                    return line.split('=')[1].rstrip()
        return "undefined"            

    def update_temperatures(self):
        # Get the current temperatures

        # TODO: Should the temperature sensor or the entire subproc. cmd line be configurable?
        temp_string_mdegC = subprocess.check_output(
            ["cat", "/sys/class/thermal/thermal_zone0/temp"])
        self.status_msg.temperatures[0] = float(temp_string_mdegC) / 1000.0

        # Check for temperature warnings and do thermal throttling
        throttle_ratio_min = 1.0
        for i, t in enumerate(self.status_msg.temperatures):
            if (t > self.system_defs_msg.critical_temps[i]):
                # Critical implies high
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = True
                self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                # Make sure to send a user string
                self.add_info_string(
                    WarningFlags.CRITICAL_TEMPERATURE_STRING, StampedString.PRI_HIGH)
                # Set the throttle ratio to 0% globally
                rospy.logerr_throttle(
                    10, "%s: temperature = %f", WarningFlags.CRITICAL_TEMPERATURE_STRING, t)
                throttle_ratio_min = 0.0
            else:
                self.status_msg.warnings.flags[WarningFlags.CRITICAL_TEMPERATURE] = False
                if (t > self.system_defs_msg.warning_temps[i]):
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = True
                    throttle_ratio_i = 1.0 - ((t - self.system_defs_msg.warning_temps[i]) / (
                        self.system_defs_msg.critical_temps[i] - self.system_defs_msg.warning_temps[i]))
                    #rospy.logwarn_throttle( 10, "%s: temperature = %f", WarningFlags.HIGH_TEMPERATURE_STRING, t)
                    throttle_ratio_min = min(
                        throttle_ratio_i, throttle_ratio_min)
                else:
                    self.status_msg.warnings.flags[WarningFlags.HIGH_TEMPERATURE] = False
            # If a new thermal throttle ratio was computed, publish it globally
            if (throttle_ratio_min != self.current_throttle_ratio):
                self.throttle_ratio_pub.publish(Float32(throttle_ratio_min))
                self.current_throttle_ratio = throttle_ratio_min
                #rospy.logwarn("New thermal rate throttle value: %f%%", self.current_throttle_ratio)

    def update_storage(self):
        # Data partition
        try:
            statvfs = os.statvfs(self.storage_mountpoint)
        except Exception as e:
            warn_str = "Error checking data storage status of " + self.storage_mountpoint + ": " + e.what()
            rospy.logwarn(warn_str)
            self.add_info_string("warn_str")
            self.status_msg.disk_usage = 0
            self.storage_rate = 0
            return

        disk_free = float(statvfs.f_frsize) * \
            statvfs.f_bavail / BYTES_PER_MEGABYTE  # In MB
        self.status_msg.disk_usage = self.system_defs_msg.disk_capacity - disk_free

        self.disk_usage_deque.append(self.status_msg.disk_usage)
        self.status_msg.storage_rate = (
            self.disk_usage_deque[-1] - self.disk_usage_deque[0]) / (len(self.disk_usage_deque)*self.STATUS_PERIOD)

        # Check for disk warnings
        if (disk_free < self.DISK_FULL_MARGIN_MB):
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = True
            self.add_info_string("Max disk usage exceeded",
                                 StampedString.PRI_HIGH)
            # Force all nodes to stop data saving
            self.save_data_pub.publish(save_continuous=False, save_raw=False)
        else:
            self.status_msg.warnings.flags[WarningFlags.DISK_FULL] = False

    def provide_sw_update_status(self, req):
        resp = SystemSoftwareStatusQueryResponse()
        resp.new_sys_img_staging_device = self.get_device_friendly_name(self.new_img_staging_device)
        resp.new_sys_img_staging_device_free_mb = sw_update_utils.getPartitionFreeByteCount(self.new_img_staging_device) / BYTES_PER_MEGABYTE

        # Don't query anything if we are in the middle of installing a new image
        if self.installing_new_image:
            resp.new_sys_img = 'busy'
            resp.new_sys_img_version = 'busy'
            return resp
        
        # At this point, not currently installing, so clear any previous query failed message so the status update logic below will work
        self.status_msg.sys_img_update_status = ""

        (status, err_string, new_img_file, new_img_version, new_img_filesize) = sw_update_utils.checkForNewImageAvailable(
            self.new_img_staging_device, self.new_img_staging_device_removable)
        if status is False:
            rospy.logwarn("Unable to update software status: " + err_string)
            resp.new_sys_img = 'query failed'
            resp.new_sys_img_version = 'query failed'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = 'query failed'
            return resp
        
        # Update the response
        if new_img_file:
            resp.new_sys_img = new_img_file
            resp.new_sys_img_version = new_img_version
            resp.new_sys_img_size_mb = new_img_filesize / BYTES_PER_MEGABYTE
            self.status_msg.sys_img_update_status = "ready to install"
        else:
            resp.new_sys_img = 'none detected'
            resp.new_sys_img_version = 'none detected'
            resp.new_sys_img_size_mb = 0
            self.status_msg.sys_img_update_status = "no new image available"
                
        return resp
    
    def provide_system_data_folder(self, req):
        response = SystemStorageFolderQueryResponse()
        if req.type not in self.storage_subdirs:
            response.folder_path = None
        else:
            response.folder_path = self.storage_subdirs[req.type]
        return response

    def provide_driver_folder(self, req):
        return self.DRIVERS_PATH

    def publish_periodic_status(self, event):
        self.status_msg.sys_time = event.current_real

        # Populate the rest of the message contents
        # Temperature(s)
        self.update_temperatures()

        # Disk usage
        self.update_storage()

        # Now publish it
        self.status_pub.publish(self.status_msg)

        # Always clear info strings after publishing
        del self.status_msg.info_strings[:]

    def set_save_status(self, save_msg):
        self.status_msg.save_all_enabled = save_msg.save_continuous

    def ensure_reqd_storage_subdirs(self):
        # First check that the storage partition is actually mounted
        if not os.path.ismount(self.storage_mountpoint):
           rospy.logwarn("NEPI Storage partition is not mounted... attempting to mount")
           ret, msg = sw_update_utils.mountPartition(self.nepi_storage_device, self.storage_mountpoint)
           if ret is False:
               rospy.logwarn("Unable to mount NEPI Storage partition... system may be dysfunctional")
               #return False # Allow it continue on local storage...

        # ... as long as there is enough space
        self.update_storage()
        if self.status_msg.warnings.flags[WarningFlags.DISK_FULL] is True:
            rospy.logerr("Insufficient space on storage partition")
            self.storage_mountpoint = ""
            return False

        # Gather owner and group details for storage mountpoint
        stat_info = os.stat(self.storage_mountpoint)
        self.storage_uid = stat_info.st_uid
        self.storage_gid = stat_info.st_gid

        # Check for and create subdirectories as necessary
        for subdir in self.REQD_STORAGE_SUBDIRS:
            full_path_subdir = os.path.join(self.storage_mountpoint, subdir)
            if not os.path.isdir(full_path_subdir):
                rospy.logwarn("Required storage subdir " + subdir + " not present... will create")
                os.makedirs(full_path_subdir)
            
            # And set the owner:group and permissions. Do this every time to fix bad settings e.g., during SSD setup
            # TODO: Different owner:group for different folders?
            os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + full_path_subdir) # Use os.system instead of os.chown to have a recursive option
            #os.chown(full_path_subdir, self.storage_uid, self.storage_gid)
            os.system('chmod -R 0775 ' + full_path_subdir)
            self.storage_subdirs[subdir] = full_path_subdir
        # Do the same for the Drivers Folder
        if not os.path.isdir(self.DRIVERS_PATH):
                rospy.logwarn("Driver folder " + self.DRIVERS_PATH + " not present... will create")
                os.makedirs(self.DRIVERS_PATH)
        os.system('chown -R ' + str(self.storage_uid) + ':' + str(self.storage_gid) + ' ' + self.DRIVERS_PATH) # Use os.system instead of os.chown to have a recursive option
        os.system('chmod -R 0775 ' + self.DRIVERS_PATH)
        self.storage_subdirs['drivers'] = self.DRIVERS_PATH
        return True

    def clear_data_folder(self, msg):
        if (self.status_msg.save_all_enabled is True):
            rospy.logwarn(
                "Refusing to clear data folder because data saving is currently enabled")
            return

        rospy.loginfo("Clearing data folder by request")
        data_folder = self.storage_subdirs['data']
        if not os.path.isdir(data_folder):
            rospy.logwarn(
                "No such folder " + data_folder + "... nothing to clear"
            )
            return

        for filename in os.listdir(data_folder):
            file_path = os.path.join(data_folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                rospy.logwarn('Failed to delete %s. Reason: %s' %
                              (file_path, e))

    def set_op_environment(self, msg):
        if (msg.data != OpEnvironmentQueryResponse.OP_ENV_AIR) and (msg.data != OpEnvironmentQueryResponse.OP_ENV_WATER):
            rospy.logwarn(
                "Setting environment parameter to a non-standard value: %s", msg.data)
        rospy.set_param("~op_environment", msg.data)

    def set_device_id(self, msg):
        # First, validate the characters in the msg as namespace chars -- blank string is okay here to clear the value
        if (msg.data) and (not self.valid_device_id_re.match(msg.data)):
            rospy.logerr("Invalid device ID: %s", msg.data)
            return

        # Otherwise, overwrite the DEVICE_ID in sys_env.bash
        file_lines = []
        with open(self.SYS_ENV_PATH, "r") as f:
            for line in f:
                if line.startswith("export DEVICE_ID"):
                    file_lines.append("export DEVICE_ID=" + msg.data + '\n')
                else:
                    file_lines.append(line)
        tmp_filename = self.SYS_ENV_PATH + ".tmp"
        with open(tmp_filename, "w") as f:
            for line in file_lines:
                f.write(line)

        # Now overwrite the original file as an autonomous operation
        os.rename(tmp_filename, self.SYS_ENV_PATH)
        rospy.logwarn("Device ID Updated - Requires device reboot")
        self.add_info_string(
            "Device ID updated - Requires device reboot", StampedString.PRI_ELEVATED)

    def handle_system_error_msg(self, msg):
        self.add_info_string(msg.data, StampedString.PRI_HIGH)

    def receive_sw_update_progress(self, progress_val):
        self.status_msg.sys_img_update_progress = progress_val

    def receive_archive_progress(self, progress_val):
        self.status_msg.sys_img_archive_progress = progress_val
    
    def handle_install_new_img(self, msg):
        if self.installing_new_image:
            rospy.logwarn("New image is already being installed")
            return

        decompressed_img_filename = msg.data
        self.status_msg.sys_img_update_status = 'flashing'
        self.installing_new_image = True

        status, err_msg = sw_update_utils.writeImage(self.new_img_staging_device, decompressed_img_filename, self.inactive_rootfs_device, 
                                                     do_slow_transfer=False, progress_cb=self.receive_sw_update_progress)

        # Finished installing
        self.installing_new_image = False
        if status is False:
            rospy.logerr("Failed to flash image: " + err_msg)
            self.status_msg.sys_img_update_status = 'failed'
            return
        else:
            rospy.loginfo("Finished flashing new image to inactive partition")
            self.status_msg.sys_img_update_status = 'complete - needs rootfs switch and reboot'

        # Check and repair the newly written filesystem as necessary
        status, err_msg = sw_update_utils.checkAndRepairPartition(self.inactive_rootfs_device)
        if status is False:
            rospy.logerr("Newly flashed image has irrepairable filesystem issues: ", err_msg)
            self.status_msg.sys_img_update_status = 'failed - fs errors'
            return
        else:
            rospy.loginfo("New image filesystem checked and repaired (as necessary)")

        # Do automatic rootfs switch if so configured
        if self.auto_switch_rootfs_on_new_img_install:
            if self.rootfs_ab_scheme == 'nepi':
                status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(self.first_stage_rootfs_device)
            elif self.rootfs_ab_scheme == 'jetson':
                status, err_msg = sw_update_utils.switchActiveAndInactivePartitionsJetson()
            else:
                err_msg = "Unknown ROOTFS A/B Scheme"
                status = False
                
            if status is False:
                rospy.logwarn("Automatic rootfs active/inactive switch failed: " + err_msg)
            else:
                rospy.loginfo("Executed automatic rootfs A/B switch... on next reboot new image will load")
                self.status_msg.warnings.flags[WarningFlags.ACTIVE_INACTIVE_ROOTFS_STALE] = True
                self.status_msg.sys_img_update_status = 'complete - needs reboot'
    
    def handle_switch_active_inactive_rootfs(self, msg):
        if self.rootfs_ab_scheme == 'nepi':
            status, err_msg = sw_update_utils.switchActiveAndInactivePartitions(self.first_stage_rootfs_device)
        elif self.rootfs_ab_scheme == 'jetson':
            status, err_msg = sw_update_utils.switchActiveAndInactivePartitionsJetson()
        else:
            err_msg = "Unknown ROOTFS A/B Scheme"
            status = False
            
        if status is False:
            rospy.logwarn("Failed to switch active/inactive rootfs: " + err_msg)
            return

        self.status_msg.warnings.flags[WarningFlags.ACTIVE_INACTIVE_ROOTFS_STALE] = True
        rospy.logwarn(
            "Switched active and inactive rootfs. Must reboot system for changes to take effect")

    def handle_archive_inactive_rootfs(self, msg):
        if self.archiving_inactive_image is True:
            rospy.logwarn("Already in the process of archiving image")
            return

        now = datetime.now()
        backup_file_basename = 'nepi_rootfs_archive_' + now.strftime("%Y_%m_%d_%H%M%S") + '.img.raw'
        self.status_msg.sys_img_archive_status = 'archiving'
        self.status_msg.sys_img_archive_filename = backup_file_basename

        # Transfers to USB seem to have trouble with the standard block size, so allow those to proceed at a lower
        # block size
        slow_transfer = True if self.usb_device in self.new_img_staging_device else False
                
        self.archiving_inactive_image = True
        status, err_msg = sw_update_utils.archiveInactiveToStaging(
            self.inactive_rootfs_device, self.new_img_staging_device, backup_file_basename, slow_transfer, progress_cb = self.receive_archive_progress)
        self.archiving_inactive_image = False

        if status is False:
            rospy.logerr("Failed to backup inactive rootfs: " + err_msg)
            self.status_msg.sys_img_archive_status = 'failed'
        else:
            rospy.loginfo("Finished archiving inactive rootfs")
            self.status_msg.sys_img_archive_status = 'archive complete'
    
    def provide_system_defs(self, req):
        return SystemDefsQueryResponse(self.system_defs_msg)

    def provide_op_environment(self, req):
        # Just proxy the param server
        return OpEnvironmentQueryResponse(rospy.get_param("~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR))
    
    def save_data_prefix_callback(self, msg):
        save_data_prefix = msg.data

        data_folder = self.storage_subdirs['data']
        if data_folder is None:
            return # No data directory

        # Now ensure the directory exists if this prefix defines a subdirectory
        full_path = os.path.join(data_folder, save_data_prefix)
        parent_path = os.path.dirname(full_path)
        if not os.path.exists(parent_path):
            rospy.loginfo("Creating new data subdirectory " + parent_path)
            os.makedirs(parent_path)
            
            # Gather owner and group details for data folder to propagate them
            # TODO: Do we need to do this recursively in case this we are creating multiple levels of subdirectory here
            stat_info = os.stat(data_folder)
            new_dir_uid = stat_info.st_uid
            new_dir_guid = stat_info.st_gid

            os.chown(parent_path, new_dir_uid, new_dir_guid)
    
    def get_device_friendly_name(self, devfs_name):
        # Leave space for the partition number
        friendly_name = devfs_name.replace(self.emmc_device, "EMMC Partition ")
        friendly_name = friendly_name.replace(self.usb_device, "USB Partition ")
        friendly_name = friendly_name.replace(self.sd_card_device, "SD Partition ")
        friendly_name = friendly_name.replace(self.ssd_device, "SSD Partition ")
        return friendly_name

    def init_msgs(self):
        self.system_defs_msg.firmware_version = self.get_fw_rev()

        self.system_defs_msg.device_sn = self.get_device_sn()

        self.system_defs_msg.device_type = self.get_device_type()

        # TODO: Determine how many temperature readings we have. On Jetson, for example
        #      there are 8 "thermal zones" in /sys/class/thermal/
        self.system_defs_msg.temperature_sensor_names.append('CPU Zone 0')
        # TODO: Configurable warning/error temperatures
        self.system_defs_msg.warning_temps.append(60.0)
        self.system_defs_msg.critical_temps.append(70.0)

        statvfs = os.statvfs(self.storage_mountpoint)
        self.system_defs_msg.disk_capacity = statvfs.f_frsize * statvfs.f_blocks / \
            BYTES_PER_MEGABYTE     # Size of data filesystem in Megabytes

        # Gather some info about ROOTFS A/B configuration
        status = False
        if self.rootfs_ab_scheme == 'nepi':
            self.system_defs_msg.first_stage_rootfs_device = self.get_device_friendly_name(self.first_stage_rootfs_device)
            (status, err_msg, rootfs_ab_settings_dict) = sw_update_utils.getRootfsABStatus(
                self.first_stage_rootfs_device)
        elif self.rootfs_ab_scheme == 'jetson':
            self.system_defs_msg.first_stage_rootfs_device = 'N/A'
            (status, err_msg, rootfs_ab_settings_dict) = sw_update_utils.getRootfsABStatusJetson()
        else:
            rospy.logerr("Failed to identify the ROOTFS A/B Scheme... cannot update A/B info and status")

        if status is True:
            self.system_defs_msg.active_rootfs_device = self.get_device_friendly_name(rootfs_ab_settings_dict[
                'active_part_device'])

            self.system_defs_msg.active_rootfs_size_mb = sw_update_utils.getPartitionByteCount(rootfs_ab_settings_dict[
                'active_part_device']) / BYTES_PER_MEGABYTE
            
            self.inactive_rootfs_device = rootfs_ab_settings_dict[
                'inactive_part_device']
            self.system_defs_msg.inactive_rootfs_device = self.get_device_friendly_name(self.inactive_rootfs_device)

            self.system_defs_msg.inactive_rootfs_size_mb = sw_update_utils.getPartitionByteCount(self.inactive_rootfs_device) / BYTES_PER_MEGABYTE
            
            self.system_defs_msg.inactive_rootfs_fw_version = rootfs_ab_settings_dict[
                'inactive_part_fw_version']
            self.system_defs_msg.max_boot_fail_count = rootfs_ab_settings_dict[
                'max_boot_fail_count']
        else:
            rospy.logwarn(
                "Unable to gather ROOTFS A/B system definitions: " + err_msg)
            self.system_defs_msg.active_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_device = "Unknown"
            self.inactive_rootfs_device = "Unknown"
            self.system_defs_msg.inactive_rootfs_fw_version = "Unknown"
            self.system_defs_msg.max_boot_fail_count = 0

        for i in self.system_defs_msg.temperature_sensor_names:
            self.status_msg.temperatures.append(0.0)

	    # TODO: Should this be queried somehow e.g., from the param server
        self.status_msg.save_all_enabled = False

    def getNEPIStorageDevice(self):
        # Try to read the NEPI storage device out of /etc/fstab
        if os.path.exists('/etc/fstab'):
            with open('/etc/fstab', 'r') as fstab:
                lines = fstab.readlines()
                for line in lines:
                    if self.storage_mountpoint in line and not line.startswith('#'):
                        candidate = line.split()[0] # First token is the device
                        if candidate.startswith('/dev/'):
                            self.nepi_storage_device = candidate
                            rospy.loginfo('Identified NEPI storage device ' + self.nepi_storage_device + ' from /etc/fstab')
                            return
                        else:
                            rospy.logwarn('Candidate NEPI storage device from /etc/fstab is of unexpected form: ' + candidate)
            
        # If we get here, failed to get the storage device from /etc/fstab
        rospy.logwarn('Failed to get NEPI storage device from /etc/fstab -- falling back to system_mgr config file')
        if not rospy.has_param("~nepi_storage_device"):
            rospy.logerr("Parameter nepi_storage_device not available -- falling back to hard-coded " + self.nepi_storage_device)
        else:
            self.nepi_storage_device = rospy.get_param(
                "~nepi_storage_device", self.nepi_storage_device)
            rospy.loginfo("Identified NEPI storage device " + self.nepi_storage_device + ' from config file')
    
    def updateFromParamServer(self):
        op_env = rospy.get_param(
            "~op_environment", OpEnvironmentQueryResponse.OP_ENV_AIR)
        # Publish it to all subscribers (which includes this node) to ensure the parameter is applied
        self.set_op_env_pub.publish(String(op_env))

        # Now gather all the params and set members appropriately
        self.storage_mountpoint = rospy.get_param(
            "~storage_mountpoint", self.storage_mountpoint)
        
        self.auto_switch_rootfs_on_new_img_install = rospy.get_param(
            "~auto_switch_rootfs_on_new_img_install", self.auto_switch_rootfs_on_new_img_install
        )

        self.first_stage_rootfs_device = rospy.get_param(
            "~first_stage_rootfs_device", self.first_stage_rootfs_device
        )

        # nepi_storage_device has some additional logic
        self.getNEPIStorageDevice()
        
        self.new_img_staging_device = rospy.get_param(
            "~new_img_staging_device", self.new_img_staging_device
        )

        self.new_img_staging_device_removable = rospy.get_param(
            "~new_img_staging_device_removable", self.new_img_staging_device_removable
        )

        self.emmc_device = rospy.get_param(
            "~emmc_device", self.emmc_device
        )

        self.usb_device = rospy.get_param(
            "~usb_device", self.usb_device
        )

        self.sd_card_device = rospy.get_param(
            "~sd_card_device", self.sd_card_device
        )

        self.ssd_device = rospy.get_param(
            "~ssd_device", self.ssd_device
        )
    
    def run(self):
        # Want to update the op_environment (from param server) through the whole system once at
        # start-up, but the only reasonable way to do that is to delay long enough to let all nodes start
        rospy.sleep(3)
        self.updateFromParamServer()

        # Reset the A/B rootfs boot fail counter -- if this node is running, pretty safe bet that we've booted successfully
        # This should be redundant, as we need a non-ROS reset mechanism, too, in case e.g., ROS nodes are delayed waiting
        # for a remote ROS master to start. That could be done in roslaunch.sh or a separate start-up script.
        if self.rootfs_ab_scheme == 'nepi': # The 'jetson' scheme handles this itself
            status, err_msg = sw_update_utils.resetBootFailCounter(
                self.first_stage_rootfs_device)
            if status is False:
                rospy.logerr("Failed to reset boot fail counter: " + err_msg)

        rospy.Timer(rospy.Duration(self.STATUS_PERIOD),
                    self.publish_periodic_status)

        # Call the method to update s/w status once internally to prime the status fields now that we have all the parameters
        # established
        self.provide_sw_update_status(0) # Any argument is fine here as the req. field is unused
        
        rospy.spin()

    def __init__(self):
        rospy.loginfo("Starting " + self.NODE_NAME + "node")
        rospy.init_node(self.NODE_NAME)

        status_period = rospy.Duration(1)  # TODO: Configurable rate?

        # Announce published topics
        self.status_pub = rospy.Publisher(
            'system_status', SystemStatus, queue_size=1)
        self.store_params_pub = rospy.Publisher(
            'store_params', String, queue_size=10)
        self.throttle_ratio_pub = rospy.Publisher(
            'apply_throttle', Float32, queue_size=3)
        self.set_op_env_pub = rospy.Publisher(
            'set_op_environment', String, queue_size=3)
        # For auto-stop of save data; disk full protection
        self.save_data_pub = rospy.Publisher(
            'save_data', SaveData, queue_size=1)

        self.current_throttle_ratio = 1.0

        # Subscribe to topics
        rospy.Subscriber('save_data', SaveData, self.set_save_status)
        rospy.Subscriber('clear_data_folder', Empty, self.clear_data_folder)
        rospy.Subscriber('set_op_environment', String, self.set_op_environment)

        rospy.Subscriber('set_device_id', String,
                         self.set_device_id)  # Public ns

        rospy.Subscriber('submit_system_error_msg', String,
                         self.handle_system_error_msg)

        rospy.Subscriber('install_new_image', String,
                         self.handle_install_new_img, queue_size=1)

        rospy.Subscriber('switch_active_inactive_rootfs', Empty,
                         self.handle_switch_active_inactive_rootfs)

        rospy.Subscriber('archive_inactive_rootfs', Empty, self.handle_archive_inactive_rootfs, queue_size=1)

        rospy.Subscriber('save_data_prefix', String, self.save_data_prefix_callback)

        # Advertise services
        rospy.Service('system_defs_query', SystemDefsQuery,
                      self.provide_system_defs)
        rospy.Service('op_environment_query', OpEnvironmentQuery,
                      self.provide_op_environment)
        rospy.Service('sw_update_status_query', SystemSoftwareStatusQuery,
                       self.provide_sw_update_status)

        self.save_cfg_if = SaveCfgIF(
            updateParamsCallback=None, paramsModifiedCallback=self.updateFromParamServer)

        # Need to get the storage_mountpoint and first-stage rootfs early because they are used in init_msgs()
        self.storage_mountpoint = rospy.get_param(
            "~storage_mountpoint", self.storage_mountpoint)
        self.first_stage_rootfs_device = rospy.get_param(
            "~first_stage_rootfs_device", self.first_stage_rootfs_device)
        
        # Need to identify the rootfs scheme because it is used in init_msgs()
        self.rootfs_ab_scheme = sw_update_utils.identifyRootfsABScheme()

        self.init_msgs()

        # Ensure that the user partition is properly laid out
        self.storage_subdirs = {} # Populated in function below
        if self.ensure_reqd_storage_subdirs() is True:
            # Now can advertise the system folder query
            rospy.Service('system_storage_folder_query', SystemStorageFolderQuery,
                self.provide_system_data_folder)

        self.valid_device_id_re = re.compile(r"^[a-zA-Z][\w]*$")

        self.run()


if __name__ == '__main__':
    SysMgr = SystemMgrNode()
