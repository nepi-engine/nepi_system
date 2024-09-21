/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef NAV_POS_MGR_H
#define NAV_POS_MGR_H

#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

#include "sdk_node.h"
#include "save_data_interface.h"
#include "algorithms/approx_moving_avg.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "nepi_ros_interfaces/SaveData.h"
#include "nepi_ros_interfaces/Offset.h"
#include "nepi_ros_interfaces/Heading.h"
#include "nepi_ros_interfaces/NavPoseQuery.h"
#include "nepi_ros_interfaces/NavPoseStatusQuery.h"

namespace Numurus
{
class SaveDataInterface;

class NavPoseMgr : public SDKNode
{
public:
	NavPoseMgr();
	virtual ~NavPoseMgr();

	// Inherited from SDKNode
	void init() override;
	void retrieveParams() override;
	void initServices() override;
	void initSubscribers() override;
	void initPublishers() override;

private:
	NodeParam<std::string> gps_fix_topic;
	NodeParam<float> init_lat_deg;
	NodeParam<float> init_lon_deg;
	NodeParam<float> init_alt_m_hae;
	
	NodeParam<std::string> orientation_topic;
	NodeParam<float> init_position_x;
	NodeParam<float> init_position_y;
	NodeParam<float> init_position_z;
	NodeParam<float> init_orientation_x;
	NodeParam<float> init_orientation_y;
	NodeParam<float> init_orientation_z;
	NodeParam<float> init_orientation_w;
	// TODO: init twist?	

	NodeParam<std::string> heading_topic;
	NodeParam<float> init_heading_deg;
	
	NodeParam<std::string> ahrs_out_frame_id;
	NodeParam<float> ahrs_x_offset_m;
	NodeParam<float> ahrs_y_offset_m;
	NodeParam<float> ahrs_z_offset_m;
	NodeParam<float> ahrs_x_rot_offset_deg;
	NodeParam<float> ahrs_y_rot_offset_deg;
	NodeParam<float> ahrs_z_rot_offset_deg;
	NodeParam<float> ahrs_heading_offset_deg;

	NodeParam<bool> soft_sync_time_to_gps_topic;
	NodeParam<float> max_gps_clock_drift_s;

	NodeParam<bool> broadcast_odom_tf;

	sensor_msgs::NavSatFix latest_nav_sat_fix;
	nav_msgs::Odometry latest_odometry; // Generic enough to work for orientation-only data, too.
	nepi_ros_interfaces::Heading latest_heading;

	ros::Subscriber gps_fix_sub;
	ros::Subscriber orientation_sub;
	ros::Subscriber heading_sub;

	ros::Publisher set_time_pub;
		
	SaveDataInterface *save_data_if = nullptr;

	std::string ahrs_src_frame_id;
	tf::TransformListener transform_listener;
	tf2_ros::StaticTransformBroadcaster ahrs_offsets_broadcaster;
	tf::TransformBroadcaster ahrs_pose_broadcaster;

	bool need_new_data_file = true;
	FILE *data_fd = nullptr;

	ApproxMovingAvg gps_fix_rate_calculator;
	double avg_gps_fix_rate_hz = 0.0;

	ApproxMovingAvg orientation_rate_calculator;
	double avg_orientation_rate_hz = 0.0;

	ApproxMovingAvg heading_rate_calculator;
	double avg_heading_rate_hz = 0.0;

	bool checkForNewFile = true;

	bool provideNavPose(nepi_ros_interfaces::NavPoseQuery::Request &req, nepi_ros_interfaces::NavPoseQuery::Response &resp);
	bool provideNavPoseStatus(nepi_ros_interfaces::NavPoseStatusQuery::Request &req, nepi_ros_interfaces::NavPoseStatusQuery::Response &resp);

	void setGPSFixTopicHandler(const std_msgs::String::ConstPtr &msg);
	void gpsFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg);
	void setInitGPSFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg);
	
	void setOrientationTopicHandler(const std_msgs::String::ConstPtr &msg);
	void setOrientationTopic(const std::string &topic);
	void odomHandler(const nav_msgs::Odometry::ConstPtr &msg);
	void imuHandler(const sensor_msgs::Imu::ConstPtr &msg);
	void setInitOrientationHandler(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
	
	void setHeadingTopicHandler(const std_msgs::String::ConstPtr &msg);
	void headingHandler(const std_msgs::Float64::ConstPtr &msg);
	void setInitHeadingHandler(const std_msgs::Float64::ConstPtr &msg);
	
	void setAHRSOutputFrameHandler(const std_msgs::String::ConstPtr &msg);
	void setAHRSOffsetHandler(const nepi_ros_interfaces::Offset::ConstPtr &msg);
	void enableGPSClockSyncHandler(const std_msgs::Bool::ConstPtr &msg);

	bool transformOdomData(const nav_msgs::Odometry &odom_in, nav_msgs::Odometry &odom_out);
	void setupStaticAHRSTransform(bool force_override = false);
	void broadcastLatestOdomAsTF();

	void saveDataIfNecessary();
	void startNewDataFile();
	void writeDataToFile();
	void closeDataFile();
	void saveNewData();
	void reinitHandler(const std_msgs::Empty::ConstPtr &msg);
	
	void saveDataHandler(const nepi_ros_interfaces::SaveData::ConstPtr &msg);
	void snapshotTriggerHandler(const std_msgs::Empty::ConstPtr &msg);
}; // class NavPoseTimeMgr
} // namespace Numurus
#endif // NAV_POS_TIME_MGR
