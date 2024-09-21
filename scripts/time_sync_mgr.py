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
import os.path
from shutil import copyfile
import re
import errno
import subprocess
import sys

import rospy

from std_msgs.msg import String, Empty, Time
from std_srvs.srv import Empty as EmptySrv

from nepi_ros_interfaces.msg import Reset
from nepi_ros_interfaces.msg import TimeStatus

from nepi_ros_interfaces.srv import TimeStatusQuery, TimeStatusQueryResponse

FACTORY_CFG_SUFFIX = '.num_factory'
USER_CFG_SUFFIX = '.user'

CHRONY_CFG_LINKNAME = '/etc/chrony/chrony.conf'
CHRONY_CFG_BASENAME = '/opt/nepi/config/etc/chrony/chrony.conf'
CHRONY_SYSTEMD_SERVICE_NAME = 'chrony.service'

g_last_set_time = rospy.Time(0.0)
g_sys_time_updated_pub = None
g_ntp_first_sync_time = None
g_ntp_status_check_timer = None

def symlink_force(target, link_name):
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            rospy.logerr("Unable to create symlink %s for %s", link_name, target)
            return False
    
    return True

def ensure_user_conf():
    userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
    if (False == os.path.isfile(userconf_path)):
        # Need to create it from a copy of the factory config
        factoryconf_path = CHRONY_CFG_BASENAME + FACTORY_CFG_SUFFIX
        try:
            copyfile(factoryconf_path, userconf_path)
        except:
            rospy.logerr("Unable to copy %s to %s", factoryconf_path, userconf_path)
            return False

    return symlink_force(userconf_path, CHRONY_CFG_LINKNAME)

def restart_systemd_service(service_name):
    subprocess.call(["systemctl", "restart", service_name])

def reset_to_factory_conf():
    userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
    factoryconf_path = CHRONY_CFG_BASENAME + FACTORY_CFG_SUFFIX

    symlink_force(factoryconf_path, CHRONY_CFG_LINKNAME)
    if (True == os.path.isfile(userconf_path)):
        os.remove(userconf_path)
        rospy.loginfo("Removed user config %s", userconf_path)

    # Restart chrony to allow changes to take effect
    restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

def add_server(server_host):
    if (False == ensure_user_conf()):
        return

    userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX

    #ensure just a simple hostname is being added
    host = server_host.data.split()[0]

    new_server_cfg_line = 'server ' + host + ' iburst minpoll 2'
    # TODO: May one day want to user chrony option initstepslew for even earlier synchronization
    #init_slew_cfg_line = 'initstepslew 1 ' + host
    match_line = '^' + new_server_cfg_line
    file = open(userconf_path, 'r+')
    found_match = False
    for line in file.readlines():
        if re.search(match_line, line):
            rospy.loginfo("Ignoring redundant NTP server additions for %s", host)
            found_match = True
            break

    #At EOF, so just write here
    if (False == found_match):
        rospy.loginfo("Adding new NTP server %s", host)
        file.write(new_server_cfg_line + '\n')
        # Restart chrony to allow changes to take effect
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

def remove_server(server_host):
    userconf_path = CHRONY_CFG_BASENAME + USER_CFG_SUFFIX
    if (False == os.path.isfile(userconf_path)):
        rospy.loginfo("Ignoring request to remove NTP server since factory config is in use")
        return

    # Make sure the symlink points to the user config  we've already established that user cfg exists
    if (False == ensure_user_conf()):
        return

    #ensure just a simple hostname is being added
    host = server_host.data.split()[0]

    match_line = '^server ' + host + ' iburst minpoll 2'
    # Must copy the file linebyline to a tmp, then overwrite the original
    orig_file = open(userconf_path, 'r')
    tmpfile_path = userconf_path + ".tmp"
    tmp_file = open(tmpfile_path, 'w')
    found_it = False
    for line in orig_file.readlines():
        if re.search(match_line, line):
            # Don't write this line as we want to eliminate it
            rospy.loginfo("Removing NTP server %s", host)
            found_it = True
            continue
        else:
            tmp_file.write(line)

    orig_file.close()
    tmp_file.close()
    os.rename(tmpfile_path, userconf_path)

    if True == found_it:
        # Restart chrony to allow changes to take effect
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)

def reset(msg):
    if Reset.USER_RESET == msg.reset_type:
        # Nothing to do for a User Reset as config file is always up-to-date
        rospy.loginfo("Ignoring NTP user-reset NO-OP")
    elif Reset.FACTORY_RESET == msg.reset_type:
        rospy.loginfo("Restoring NTP to factory config")
        reset_to_factory_conf()
    elif Reset.SOFTWARE_RESET == msg.reset_type:
        rospy.loginfo("Executing soft reset for NTP")
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
        rospy.signal_shutdown('Shutdown by request')
    elif Reset.HARDWARE_RESET == msg.reset_type:
        rospy.loginfo("Executing hard reset for NTP")
        # TODO: Any hardware restart required?
        restart_systemd_service(CHRONY_SYSTEMD_SERVICE_NAME)
        rospy.signal_shutdown('Shutdown by request')

def gather_ntp_status_timer_cb(event):
    # Just call the implementation method. We don't care about the event payload
    gather_ntp_status()

def gather_ntp_status():
    global g_ntp_first_sync_time
    global g_ntp_status_check_timer

    #rospy.logwarn("Debug: gather_ntp_status running (first time sync = " + str(g_ntp_first_sync_time) + ")")

    chronyc_sources = subprocess.check_output(["chronyc", "sources"], text=True).splitlines()
    ntp_status = [] # List of lists
    for line in chronyc_sources[1:]:
        if re.search('^\^|#', line): # Find sources lines by their leading "Mode" indicator
            tokens = line.split()
            source = tokens[1]
            currently_syncd = ('*' in tokens[0]) or ('+' in tokens[0])
            last_sync = tokens[5]
            current_offset = tokens[6].split('[')[0] # The string has two parts
            ntp_status.append((source, currently_syncd, last_sync, current_offset))
            if (g_ntp_first_sync_time is None) and (currently_syncd is True):
                rospy.loginfo("NTP sync first detected... publishing on sys_time_update")
                g_ntp_first_sync_time = rospy.get_rostime()
                informClockUpdate()

                # Update the RTC with this "better" clock source
                rospy.loginfo("Updating hardware clock with NTP time")
                subprocess.call(['hwclock', '-w'])

                # No longer need to run the timer
                # TODO: Should we continue to run timer to monitor for big changes, additional syncs, etc. to
                # inform the rest of the system about the time update?
                g_ntp_status_check_timer.shutdown()

    return ntp_status

def handle_time_status_query(req):
    time_status = TimeStatus()
    time_status.current_time = rospy.get_rostime()

    # Get Last PPS time from the sysfs node
    #pps_exists = os.path.isfile('/sys/class/pps/pps0/assert')
    pps_exists = False # Hard code if for now, since Jetson isn't defining /sys/class/pps -- we may never actually use PPS
    if pps_exists:
        pps_string = subprocess.check_output(["cat", "/sys/class/pps/pps0/assert"], text=True)
        pps_tokens = pps_string.split('#')
        if (len(pps_tokens) >= 2):
            time_status.last_pps = rospy.Time(float(pps_string.split('#')[0]))
        else:
            time_status.last_pps = rospy.Time(0)
            rospy.logwarn_throttle(300, "Unable to parse /sys/class/pps/pps0/assert");
    else: # Failed to find the assert file - just return no PPS
        time_status.last_pps = rospy.Time(0)
        #rospy.logwarn_once("Unable to find /sys/class/pps/pps0/assert... PPS unavailable");

    ntp_status = gather_ntp_status()
    for status_entry in ntp_status:
        time_status.ntp_sources.append(status_entry[0])
        time_status.currently_syncd.append(status_entry[1])
        time_status.last_ntp_sync.append(status_entry[2])
        time_status.current_offset.append(status_entry[3])


    # Last set time (cheater clock sync method)
    time_status.last_set_time = g_last_set_time

    return  { 'time_status': time_status }

def set_time(msg):
    # TODO: Bounds checking?
    # Use the Linux 'date -s' command-line utility.
    rospy.loginfo("Setting time from set_time topic to %ds, %dns", msg.data.secs, msg.data.nsecs)
    timestring = '@' + str(float(msg.data.secs) + (float(msg.data.nsecs) / float(1e9)))
    subprocess.call(["date", "-s", timestring])
    global g_last_set_time
    g_last_set_time = msg.data
    new_date = subprocess.check_output(["date"], text=True)
    rospy.loginfo("Updated date: %s", new_date)

    # Update the hardware clock from this "better" clock source; helps with RTC drift
    rospy.loginfo("Updating hardware clock from set_time value")
    subprocess.call(['hwclock', '-w'])

    # And tell the rest of the system
    informClockUpdate()

    # TODO: Should we use this CTypes call into librt instead?
#    import ctypes
#    import ctypes.util
#    import time

    # /usr/include/linux/time.h:
    #
    # define CLOCK_REALTIME             0
#    CLOCK_REALTIME = 0

    # /usr/include/time.h
    #
    # struct timespec
    #  {
    #    __time_t tv_sec;        /* Seconds.  */
    #    long int tv_nsec;       /* Nanoseconds.  */
    #  };
    # Structure for the timespec arg. to librt::clock_settime()
#    class timespec(ctypes.Structure):
#        _fields_ = [("tv_sec", ctypes.c_long),
#                    ("tv_nsec", ctypes.c_long)]

    # Bring in the C-language librt
#    librt = ctypes.CDLL(ctypes.util.find_library("rt"))

    # Create a new timespec object
#    ts = timespec()
#    ts.tv_sec = int(msg.data.secs)
#    ts.tv_nsec = int(msg.data.nsecs)
#    rospy.logerr(ts.tv_sec)
#    rospy.logerr(ts.tv_nsec)

    # Now call into librt::clock_settime
#    rospy.logerr(librt.clock_settime(CLOCK_REALTIME, ctypes.byref(ts)))
#    rospy.logerr(os.strerror(ctypes.get_errno()))

def time_sync_mgr():
    rospy.init_node('time_sync_mgr', disable_signals=True)

    rospy.loginfo("Starting the Time Sync Manager node")

    # Public namespace stuff
    rospy.Subscriber('add_ntp_server', String, add_server)
    rospy.Subscriber('remove_ntp_server', String, remove_server)
    rospy.Subscriber('reset', Reset, reset)
    rospy.Subscriber('set_time', Time, set_time)

    # Private namespace stuff
    rospy.Subscriber('~reset', Reset, reset)

    rospy.Service('time_status_query', TimeStatusQuery, handle_time_status_query)

    global g_sys_time_updated_pub
    g_sys_time_updated_pub = rospy.Publisher('sys_time_updated', Empty, queue_size=3)

    # Initialize the system clock from the RTC if so configured
    # RTC will be updated whenever a "good" clock source is detected; that will control drift
    init_from_rtc = rospy.get_param("~init_time_from_rtc", True)
    if init_from_rtc is True:
        rospy.loginfo("Initializing system clock from hardware clock")
        subprocess.call(['hwclock', '-s'])
        informClockUpdate() 

    # Set up a periodic timer to check for NTP sync so we can inform the rest of the system when first sync detected
    global g_ntp_status_check_timer
    g_ntp_status_check_timer = rospy.Timer(rospy.Duration(5.0), gather_ntp_status_timer_cb)
    g_ntp_status_check_timer.run()

    rospy.spin()

def informClockUpdate():
    global g_sys_time_updated_pub
    g_sys_time_updated_pub.publish() # Make sure to inform the rest of the nodes that the system clock was updated

    # For onvif_mgr, must use a service rather than the system_time_updated topic due to limitation with onvif_mgr message subscriptions
    try:
        rospy.wait_for_service('onvif_mgr/resync_onvif_device_clocks', timeout=0.1)
        resync_srv = rospy.ServiceProxy('onvif_mgr/resync_onvif_device_clocks', EmptySrv)
        resync_srv()
    except Exception as e:
        pass

if __name__ == '__main__':
    time_sync_mgr()
