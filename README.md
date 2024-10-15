# nepi_managers
Core set of NEPI management software components

Build and Install
This repository is typically built as part of the nepi_engine_ws catkin workspace. Refer to that project's top-level README for build and install instructions.

The repository may be included in other custom catkin workspaces or built using standard CMake commands (with appropriate variable definitions provided), though it should be noted that the executables here work in concert with a complete NEPI Engine installation, so operability and functionality may be limited when building outside of the nepi_engine_ws source tree.

Branching and Tagging Strategy
In general branching, merging, tagging are all limited to the nepi_engine_ws container repository, with the present submodule repository kept as simple and linear as possible.

Contribution guidelines
Bug reports, feature requests, and source code contributions (in the form of pull requests) are gladly accepted!

Who do I talk to?
At present, all user contributions and requests are vetted by Numurus technical staff, so you can use any convenient mechanism to contact us.

# nav_pose_mgr #
This repository hosts the NEPI nav_pose_mgr ROS node to support GPS and IMU integration for NEPI devices. It also includes the submodule _nepi_gpsd_, which is a lightly modified GPSD fork to include some custom NMEA sentences and a GPSD-to-ROS bridge node to convert GPSD-served nav and pose to ROS topics and services.

# ai_detector_mgr
NEPI AI detector manager node

# ai_drivers_mgr
NEPI driver management system node

# onvif_mgr #
This repository hosts the NEPI ONVIF driver and ROS nodes to support autodetection, status, controls, and data streaming for ONVIF cameras with NEPI devices.


# ai_apps_mgr
NEPI application management system node

# automation_mgr #
This repository provides the nepi_automation_mgr ROS node and support files. The nepi_automation_mgr allows users to upload executables in the form of scripts (Python, Bash, etc.) or target-compiled binaries that can be started and stopped automatically on boot-up or at will via the NEPI ROS API. These executables are generally referred to as "automation scripts," though as noted they need not necessarily be in script form.

Scripts are uploaded to the NEPI user partition automation_scripts subdirectory, typically at /mnt/nepi_storage/automation_scripts. A collection of sample scripts is provided in the separate nepi_sample_auto_scripts repository.

Console output of running scripts is logged to the NEPI user partition, typically at /mnt/nepi_storage/logs/automation_script_logs under files named according to the script filename. The log file resets each time that particular script runs.

Static and runtime statistics for each automation script are collected by nepi_automation_mgr and can be queried through the NEPI ROS API, viewed in the NEPI RUI, etc.


