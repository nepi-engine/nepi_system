/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <cstdlib>
#include <cmath>
#include <fcntl.h>

#include <sys/stat.h>
#include "std_msgs/Time.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nav_pose_mgr.h"

#define NODE_NAME	"nav_pose_mgr"

#define MAX_NAV_POS_QUERY_DELAY	5.0 // TODO: Make this a configurable parameter?

#define DEFAULT_MAX_GPS_CLOCK_DRIFT_S 0.25

#define DEG_TO_RAD(angle_deg) ((angle_deg) * M_PI / 180.0f)
#define RAD_TO_DEG(angle_rad) ((angle_rad) * 180.0f / M_PI)

#define DEFAULT_EARTH_FIXED_FRAME_ID	"map"
#define DEFAULT_ROBOT_FIXED_BASE_FRAME_ID	"base_link"

namespace Numurus
{

static inline const char* navSatStatusToString(int8_t status)
{
	if (status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) return "NO FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_FIX) return "FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX) return "SBAS FIX";
	if (status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) return "GBAS FIX";
	return "UNKNOWN";
}

static inline const char* navSatServiceToString(int8_t service)
{
	if (service == sensor_msgs::NavSatStatus::SERVICE_GPS) return "GPS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_GLONASS) return "GLONASS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_COMPASS) return "COMPASS";
	if (service == sensor_msgs::NavSatStatus::SERVICE_GALILEO) return "GALILEO";
	return "UNKNOWN";
}

NavPoseMgr::NavPoseMgr() :
	gps_fix_topic{"gps_fix_topic", "gps_fix", this},
	init_lat_deg{"init_data/lat_deg", 0.0f, this},
	init_lon_deg{"init_data/lon_deg", 0.0f, this},
	init_alt_m_hae{"init_data/alt_m_hae", 0.0f, this},
	
	orientation_topic{"orientation_topic", "pose", this},
	init_position_x{"init_data/position_x", 0.0f, this},
	init_position_y{"init_data/position_y", 0.0f, this},
	init_position_z{"init_data/position_z", 0.0f, this},
	init_orientation_x{"init_data/orientation_x", 0.0f, this},
	init_orientation_y{"init_data/orientation_y", 0.0f, this},
	init_orientation_z{"init_data/orientation_z", 0.0f, this},
	init_orientation_w{"init_data/orientation_w", 1.0f, this},
	
	heading_topic{"heading_topic", "heading", this},
	init_heading_deg{"init_data/heading_deg", 0.0f, this},
	
	ahrs_out_frame_id{"ahrs_out_frame_id", "nepi_center_frame", this},
	ahrs_x_offset_m{"ahrs_offset/x_m", 0.0f, this},
	ahrs_y_offset_m{"ahrs_offset/y_m", 0.0f, this},
	ahrs_z_offset_m{"ahrs_offset/z_m", 0.0f, this},
	ahrs_x_rot_offset_deg{"ahrs_offset/x_rot_deg", 0.0f, this},
	ahrs_y_rot_offset_deg{"ahrs_offset/y_rot_deg", 0.0f, this},
	ahrs_z_rot_offset_deg{"ahrs_offset/z_rot_deg", 0.0f, this},
	ahrs_heading_offset_deg{"ahrs_offset/heading_offset_deg", 0.0f, this},

	soft_sync_time_to_gps_topic{"soft_sync_time_to_gps_topic", false, this},
	max_gps_clock_drift_s{"max_gps_clock_drift_s", DEFAULT_MAX_GPS_CLOCK_DRIFT_S, this},

	broadcast_odom_tf{"broadcast_odom_tf", true, this},

	gps_fix_rate_calculator{10},
	orientation_rate_calculator{10},
	heading_rate_calculator{10}
	
{
	latest_nav_sat_fix.header.seq = 0;
	latest_nav_sat_fix.header.stamp = ros::Time(0.0);
	latest_nav_sat_fix.header.frame_id = DEFAULT_ROBOT_FIXED_BASE_FRAME_ID;
	latest_nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	latest_nav_sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
	latest_nav_sat_fix.latitude = 0.0;
	latest_nav_sat_fix.longitude = 0.0;
	latest_nav_sat_fix.altitude = 0.0;
	latest_nav_sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

	latest_odometry.header.seq = 0;
	latest_odometry.header.stamp = ros::Time(0.0);
	latest_odometry.header.frame_id = DEFAULT_EARTH_FIXED_FRAME_ID;
	latest_odometry.child_frame_id = DEFAULT_ROBOT_FIXED_BASE_FRAME_ID;
	latest_odometry.pose = geometry_msgs::PoseWithCovariance(); // Zero-initialized
	latest_odometry.twist = geometry_msgs::TwistWithCovariance(); // Zero-initialized
	// TODO: Covariance initializations?

	latest_heading.header.seq = 0;
	latest_heading.header.stamp = ros::Time(0.0);
	latest_heading.header.frame_id = DEFAULT_EARTH_FIXED_FRAME_ID;
	latest_heading.heading = 0.0;

	save_data_if = new SaveDataInterface(this, &n, &n_priv);
	save_data_if->registerDataProduct("nav_pose");
}

NavPoseMgr::~NavPoseMgr()
{
	if (nullptr != save_data_if)
	{
		delete save_data_if;
	}
}

void NavPoseMgr::init()
{
	SDKNode::init(); // call the base class method first. After that config params are populated
	
	// Apply init nav/pose data - use the same callback as for reinit_solution
	std_msgs::Empty::ConstPtr msg;
	reinitHandler(msg);

	// Subscribe to configured setter API
	gps_fix_sub = n.subscribe(gps_fix_topic, 3, &NavPoseMgr::gpsFixHandler, this);
	heading_sub = n.subscribe(heading_topic, 3, &NavPoseMgr::headingHandler, this);

	// Orientation subscriber depends on topic type, so has additional logic
	const std::string topic = orientation_topic;
	setOrientationTopic(topic);
}

void NavPoseMgr::retrieveParams()
{
	SDKNode::retrieveParams();

	gps_fix_topic.retrieve();
	init_lat_deg.retrieve();
	init_lon_deg.retrieve();
	init_alt_m_hae.retrieve();
	
	orientation_topic.retrieve();
	init_position_x.retrieve();
	init_position_y.retrieve();
	init_position_z.retrieve();
	init_orientation_x.retrieve();
	init_orientation_y.retrieve();
	init_orientation_z.retrieve();
	init_orientation_w.retrieve();
	
	heading_topic.retrieve();
	init_heading_deg.retrieve();
	
	ahrs_out_frame_id.retrieve();
	ahrs_x_offset_m.retrieve();
	ahrs_y_offset_m.retrieve();
	ahrs_z_offset_m.retrieve();
	ahrs_x_rot_offset_deg.retrieve();
	ahrs_y_rot_offset_deg.retrieve();
	ahrs_z_rot_offset_deg.retrieve();
	ahrs_heading_offset_deg.retrieve();

	soft_sync_time_to_gps_topic.retrieve();
	max_gps_clock_drift_s.retrieve();
	
	broadcast_odom_tf.retrieve();
}

void NavPoseMgr::initServices()
{
	// Call the base method
	SDKNode::initServices();

	// Advertise trigger status query service
	servicers.push_back(n.advertiseService("nav_pose_query", &Numurus::NavPoseMgr::provideNavPose, this));
	servicers.push_back(n.advertiseService("nav_pose_status_query", &Numurus::NavPoseMgr::provideNavPoseStatus, this));
}

void NavPoseMgr::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	subscribers.push_back(n_priv.subscribe("set_gps_fix_topic", 3, &NavPoseMgr::setGPSFixTopicHandler, this));
	subscribers.push_back(n_priv.subscribe("set_init_gps_fix", 3, &NavPoseMgr::setInitGPSFixHandler, this));
		
	subscribers.push_back(n_priv.subscribe("set_orientation_topic", 3, &NavPoseMgr::setOrientationTopicHandler, this));
	subscribers.push_back(n_priv.subscribe("set_init_orientation", 3, &NavPoseMgr::setInitOrientationHandler, this));
	
	subscribers.push_back(n_priv.subscribe("set_heading_topic", 3, &NavPoseMgr::setHeadingTopicHandler, this));
	subscribers.push_back(n_priv.subscribe("set_init_heading", 3, &NavPoseMgr::setInitHeadingHandler, this));
		
	subscribers.push_back(n_priv.subscribe("set_ahrs_out_frame", 3, &NavPoseMgr::setAHRSOutputFrameHandler, this));
	subscribers.push_back(n_priv.subscribe("set_ahrs_offset", 3, &NavPoseMgr::setAHRSOffsetHandler, this));
	subscribers.push_back(n_priv.subscribe("enable_gps_clock_sync", 3, &NavPoseMgr::enableGPSClockSyncHandler, this));

	subscribers.push_back(n_priv.subscribe("reinit_solution", 3, &NavPoseMgr::reinitHandler, this));

	subscribers.push_back(n.subscribe("save_data", 3, &NavPoseMgr::saveDataHandler, this));
	subscribers.push_back(n_priv.subscribe("save_data", 3, &NavPoseMgr::saveDataHandler, this));
	subscribers.push_back(n.subscribe("snapshot_trigger", 3,&NavPoseMgr::snapshotTriggerHandler, this));
	subscribers.push_back(n_priv.subscribe("snapshot_trigger", 3, &NavPoseMgr::snapshotTriggerHandler, this));

}

void NavPoseMgr::initPublishers()
{
	set_time_pub = n.advertise<std_msgs::Time>("set_time", 3);
}

bool NavPoseMgr::provideNavPose(nepi_ros_interfaces::NavPoseQuery::Request &req, nepi_ros_interfaces::NavPoseQuery::Response &resp)
{
	// TODO: Honor request time by keeping data queued and interpolating as necessary
	resp.nav_pose.timestamp = ros::Time::now(); // For now just respond with the current time and latest data (which is individually time-stamped)
	
	resp.nav_pose.fix = latest_nav_sat_fix;
	resp.nav_pose.heading = latest_heading;
	
	if (true == req.transform)
	{
		nav_msgs::Odometry transformed_odom;
		resp.transformed = transformOdomData(latest_odometry, resp.nav_pose.odom);
		if (true == resp.transformed)
		{
			const float heading_offset = ahrs_heading_offset_deg;
			resp.nav_pose.heading.heading += heading_offset;
		}
	}
	else
	{
		resp.transformed = false;
		resp.nav_pose.odom = latest_odometry;
	}
	

	return true;
}

bool NavPoseMgr::provideNavPoseStatus(nepi_ros_interfaces::NavPoseStatusQuery::Request&, nepi_ros_interfaces::NavPoseStatusQuery::Response &resp)
{
	resp.status.nav_sat_fix_topic = gps_fix_sub.getTopic();
	resp.status.last_nav_sat_fix = latest_nav_sat_fix.header.stamp;
	resp.status.nav_sat_fix_rate = avg_gps_fix_rate_hz;

	
	resp.status.orientation_topic = orientation_sub.getTopic();
	resp.status.last_orientation = latest_odometry.header.stamp;
	resp.status.orientation_rate = avg_orientation_rate_hz;

	resp.status.heading_topic = heading_sub.getTopic();
	resp.status.last_heading = latest_heading.header.stamp;
	resp.status.heading_rate = avg_heading_rate_hz;

	// Finally, the current AHRS transform via transform_listener
	ros::Time latest(0.0);
	const std::string src_frame = latest_odometry.child_frame_id;
	const std::string target_frame = ahrs_out_frame_id;
	tf::StampedTransform transform;
	try
	{
		//transform_listener.lookupTransform(src_frame, target_frame, latest, transform); // API description is backwards
		transform_listener.lookupTransform(target_frame, src_frame, latest, transform);
		tf::transformStampedTFToMsg(transform, resp.status.transform);
	}
	catch(const std::exception& e)
	{
		//ROS_WARN_THROTTLE(30, "Unable to lookup transform from %s to %s for nav/pose status... will report null transform", src_frame.c_str(), target_frame.c_str());
	}

	resp.status.heading_offset = ahrs_heading_offset_deg;

	resp.status.gps_clock_sync_enabled = soft_sync_time_to_gps_topic;
	
	return true;
}

void NavPoseMgr::setGPSFixTopicHandler(const std_msgs::String::ConstPtr &msg)
{
	//TODO: Validation?
	gps_fix_topic = msg->data;
	
	gps_fix_sub.shutdown();
	gps_fix_sub = n.subscribe(gps_fix_topic, 3, &NavPoseMgr::gpsFixHandler, this);
	gps_fix_rate_calculator.reset();
	avg_gps_fix_rate_hz = 0.0;
}

void NavPoseMgr::gpsFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	// Update the average rate via the windowed averager
	const double delta_t = (msg->header.stamp - latest_nav_sat_fix.header.stamp).toSec();
	avg_gps_fix_rate_hz = 1.0 / gps_fix_rate_calculator.calculateNext(delta_t);

	// TODO: Any validation?
	latest_nav_sat_fix = *msg;

	if (true == soft_sync_time_to_gps_topic)
	{
		const ros::Duration drift = ros::Time::now() - latest_nav_sat_fix.header.stamp;
		const double drift_s = drift.toSec();
		if ((drift_s > max_gps_clock_drift_s) || (drift_s < -max_gps_clock_drift_s))
		{
			//ROS_INFO_THROTTLE(10.0, "Updating system clock to GPS topic timestamp (drift was %.3fs)", drift_s);
			std_msgs::Time time_msg;
			time_msg.data = latest_nav_sat_fix.header.stamp;
			set_time_pub.publish(time_msg);
		}
	}
	saveDataIfNecessary();
}

void NavPoseMgr::setInitGPSFixHandler(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_lat_deg = msg->latitude;
	init_lon_deg = msg->longitude;
	init_alt_m_hae = msg->altitude;
}

void NavPoseMgr::setOrientationTopicHandler(const std_msgs::String::ConstPtr &msg)
{
	setOrientationTopic(msg->data);
	orientation_rate_calculator.reset();
	avg_orientation_rate_hz = 0.0;
}


void NavPoseMgr::setOrientationTopic(const std::string &topic)
{
	// Need to do some introspection to determine topic type
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) 
	{
    	const ros::master::TopicInfo& info = *it;
		if (info.name == topic) 
		{
			if (info.datatype == "sensor_msgs/Imu")
			{
				orientation_topic = topic;
				orientation_sub.shutdown();
				orientation_sub = n.subscribe(orientation_topic, 3, &NavPoseMgr::imuHandler, this);
				return;
			}
			else if (info.datatype == "nav_msgs/Odometry")
			{
				orientation_topic = topic;
				orientation_sub.shutdown();
				orientation_sub = n.subscribe(orientation_topic, 3, &NavPoseMgr::odomHandler, this);
				return;
			}
			// TODO: geometry_msgs/Quaternion (for very simple pose publishers)?
			else
			{
				//ROS_ERROR("Cannot obtain orientation from topic %s of type %s", info.name.c_str(), info.datatype.c_str());
			}
		}
	}

	// If we get this far, the topic is not yet known to rosmaster, so just default to Odometry
	// TODO: Should we validate this periodically? What happens if IMU msg arrives on this topic?
	//ROS_WARN("Specified orientation topic %s is not yet published, so datatype cannot be determined... will assume nav_msgs/Odometry", topic.c_str());
	orientation_topic = topic;
	orientation_sub.shutdown();
	orientation_sub = n.subscribe(orientation_topic, 3, &NavPoseMgr::odomHandler, this);
}

void NavPoseMgr::odomHandler(const nav_msgs::Odometry::ConstPtr &msg)
{
	// Update the average rate via the windowed averager
	const double delta_t = (msg->header.stamp - latest_odometry.header.stamp).toSec();
	avg_orientation_rate_hz = 1.0 / orientation_rate_calculator.calculateNext(delta_t);

	// TODO: Validation?
	latest_odometry = *msg;

	// And broadcast TF if so configured
	if (true == broadcast_odom_tf)
	{
		broadcastLatestOdomAsTF();
	}

	saveDataIfNecessary();
}

void NavPoseMgr::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
	// Update the average rate via the windowed averager
	const double delta_t = (msg->header.stamp - latest_odometry.header.stamp).toSec();
	avg_orientation_rate_hz = 1.0 / orientation_rate_calculator.calculateNext(delta_t);

	// Reset odometry -- we'll fill in just those fields that we can glean from IMU message
	latest_odometry = nav_msgs::Odometry();
	latest_odometry.header = msg->header;
	latest_odometry.header.frame_id = msg->header.frame_id; // TODO: Configurable?
	latest_odometry.child_frame_id = msg->header.frame_id; 
	latest_odometry.pose.pose.orientation = msg->orientation;

	// And broadcast TF if so configured
	if (true == broadcast_odom_tf)
	{
		broadcastLatestOdomAsTF();
	}

	saveDataIfNecessary();
}

void NavPoseMgr::setInitOrientationHandler(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
	init_orientation_x = msg->quaternion.x;
	init_orientation_y = msg->quaternion.y;
	init_orientation_z = msg->quaternion.z;
	init_orientation_w = msg->quaternion.w;
}

void NavPoseMgr::setHeadingTopicHandler(const std_msgs::String::ConstPtr &msg)
{
	//TODO: Validation?
	heading_topic = msg->data;

	heading_sub.shutdown();
	heading_sub = n.subscribe(heading_topic, 3, &NavPoseMgr::headingHandler, this);
	heading_rate_calculator.reset();
	avg_heading_rate_hz = 0.0;
}

void NavPoseMgr::headingHandler(const std_msgs::Float64::ConstPtr &msg)
{
	// Update the average rate via the windowed averager
	ros::Time now = ros::Time::now();
	double delta_t = (now - latest_heading.header.stamp).toSec();
	avg_heading_rate_hz = 1.0 / heading_rate_calculator.calculateNext(delta_t);

	latest_heading.header.stamp = now; // No timestamp in the input, so just use current time
	latest_heading.header.frame_id = latest_odometry.header.frame_id; // Assume heading matches odometry source
	++latest_heading.header.seq;
	latest_heading.heading = msg->data;

	saveDataIfNecessary();
}

void NavPoseMgr::setInitHeadingHandler(const std_msgs::Float64::ConstPtr &msg)
{
	if (msg->data <= -360.0 || msg->data >= 360.0)
	{
		//ROS_ERROR("Invalid init heading: %f deg... ignoring", msg->data);
	}
	else
	{
		init_heading_deg = msg->data;
	}
}

void NavPoseMgr::setAHRSOutputFrameHandler(const std_msgs::String::ConstPtr &msg)
{
	const std::string output_frame = msg->data;
	if (ahrs_out_frame_id != output_frame)
	{
		ahrs_out_frame_id = output_frame;
		setupStaticAHRSTransform();
	}
}

void NavPoseMgr::setAHRSOffsetHandler(const nepi_ros_interfaces::Offset::ConstPtr &msg)
{
	ahrs_x_offset_m = msg->translation.x;
	ahrs_y_offset_m = msg->translation.y;
	ahrs_z_offset_m = msg->translation.z;

	ahrs_x_rot_offset_deg = msg->rotation.x;
	ahrs_y_rot_offset_deg = msg->rotation.y;
	ahrs_z_rot_offset_deg = msg->rotation.z;

	ahrs_heading_offset_deg = msg->heading;

	setupStaticAHRSTransform(true); // User is pushing new offsets, so assume they want these
}

void NavPoseMgr::enableGPSClockSyncHandler(const std_msgs::Bool::ConstPtr &msg)
{
	//ROS_INFO("%s system clock sync. from GPS", (msg->data)? "Enabling" : "Disabling");
	soft_sync_time_to_gps_topic = msg->data;
}



bool NavPoseMgr::transformOdomData(const nav_msgs::Odometry &odom_in, nav_msgs::Odometry &odom_out)
{
	const std::string output_frame_id = ahrs_out_frame_id;
	// Already in the requested frame -- don't transform
	if (output_frame_id == odom_in.child_frame_id)
	{
		return false;
	}

	odom_out = odom_in;
	
	geometry_msgs::PoseStamped pose_in, pose_out;
	pose_in.header = odom_in.header;
	pose_in.header.frame_id = odom_in.child_frame_id; // Transform should be from the odom output frame to the ahrs_out_frame... easy place to make a mistake.
	pose_in.pose = odom_in.pose.pose;
	
	geometry_msgs::Vector3Stamped linear_rates_in, linear_rates_out, angular_rates_in, angular_rates_out;
	linear_rates_in.header = odom_in.header;
	linear_rates_in.vector = odom_in.twist.twist.linear;
	angular_rates_in.header = odom_in.header;
	angular_rates_in.vector = odom_in.twist.twist.angular;
	
	try
	{
		transform_listener.transformPose(output_frame_id, pose_in, pose_out);
		transform_listener.transformVector(output_frame_id, linear_rates_in, linear_rates_out);
		transform_listener.transformVector(output_frame_id, angular_rates_in, angular_rates_out);
	}
	catch(const std::exception& e)
	{
		//ROS_WARN_THROTTLE(30, "Failed to transform odometry from %s to %s (%s)... data is unadjusted", 
		//		 odom_in.header.frame_id.c_str(), output_frame_id.c_str(), e.what());
		return false;
	}

	// If we get this far, all transforms succeeded so update the output with transformed values
	odom_out.header.frame_id = odom_in.header.frame_id;
	odom_out.child_frame_id = output_frame_id;
	odom_out.pose.pose = pose_out.pose;
	odom_out.twist.twist.linear = linear_rates_out.vector;
	odom_out.twist.twist.angular = angular_rates_out.vector;
	return true;
}

void NavPoseMgr::setupStaticAHRSTransform(bool force_override)
{
	// First, check if there is an existing transform between source and destination. If so, that takes precedence.
	const std::string src_frame = latest_odometry.child_frame_id;
	const std::string target_frame = ahrs_out_frame_id;
	const ros::Time latest(0.0);
	tf::StampedTransform transform;
	
	if (force_override == false) 
	{
		try
		{
			//transform_listener.lookupTransform(src_frame, target_frame, latest, transform); // API description is backwards
			transform_listener.lookupTransform(target_frame, src_frame, latest, transform);
			const tf::Vector3 translation = transform.getOrigin();
			const tf::Matrix3x3 mat = tf::Matrix3x3(transform.getRotation());
			double r,p,y;
			mat.getRPY(r,p,y);
			ROS_INFO("Detected existing transform from %s to %s:\n\tTranslation = [%f,%f,%f]\n\tRotation = [%f,%f,%f]... not overriding with static",
					src_frame.c_str(), target_frame.c_str(), translation.getX(), translation.getY(), translation.getZ(),
					RAD_TO_DEG(r), RAD_TO_DEG(p), RAD_TO_DEG(y));
			return;
		}
		catch(const std::exception& e)
		{
			ROS_INFO("No existing transform from %s to %s -- will create one based on configured AHRS offsets", 
					src_frame.c_str(), target_frame.c_str());
		}
	}
	
	ROS_INFO("Setting up static transform from %s to %s:\n\tTranslation = [%f,%f,%f]\n\tRotation = [%f,%f,%f]",
			src_frame.c_str(), target_frame.c_str(),
			(float)ahrs_x_offset_m, (float)ahrs_y_offset_m, (float)ahrs_z_offset_m,
			(float)ahrs_x_rot_offset_deg, (float)ahrs_y_rot_offset_deg, (float)ahrs_z_rot_offset_deg);
		
	// Build the transform
	geometry_msgs::TransformStamped static_transform;
	static_transform.header.stamp = ros::Time::now();
	static_transform.header.frame_id = ahrs_out_frame_id; // Seems backwards, but works
	static_transform.child_frame_id = src_frame; // Seems backwards, but works
	static_transform.transform.translation.x = ahrs_x_offset_m;
	static_transform.transform.translation.y = ahrs_y_offset_m;
	static_transform.transform.translation.z = ahrs_z_offset_m;
	tf2::Quaternion quat;
	quat.setRPY(DEG_TO_RAD(ahrs_x_rot_offset_deg), DEG_TO_RAD(ahrs_y_rot_offset_deg), DEG_TO_RAD(ahrs_z_rot_offset_deg));
	static_transform.transform.rotation.x = quat.x();
	static_transform.transform.rotation.y = quat.y();
	static_transform.transform.rotation.z = quat.z();
	static_transform.transform.rotation.w = quat.w();
		
	// Send the transform
	ahrs_offsets_broadcaster.sendTransform(static_transform);
	
}

void NavPoseMgr::broadcastLatestOdomAsTF()
{
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = latest_odometry.header.stamp;
	//transform.header.frame_id = latest_odometry.header.frame_id;
	//transform.child_frame_id = latest_odometry.child_frame_id;
	transform.header.frame_id = latest_odometry.child_frame_id;
	transform.child_frame_id = latest_odometry.header.frame_id;
	transform.transform.translation.x = latest_odometry.pose.pose.position.x;
	transform.transform.translation.y = latest_odometry.pose.pose.position.y;
	transform.transform.translation.z = latest_odometry.pose.pose.position.z;
	transform.transform.rotation = latest_odometry.pose.pose.orientation;

	ahrs_pose_broadcaster.sendTransform(transform);
}


void NavPoseMgr::reinitHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ros::Time now = ros::Time::now();
	// Apply init nav/pose data
	latest_nav_sat_fix.header.seq = 0;
	latest_nav_sat_fix.header.stamp = now; // Seems as good as any time to report here
	latest_nav_sat_fix.header.frame_id = DEFAULT_ROBOT_FIXED_BASE_FRAME_ID;
	// Assume purely zero'd defaults should be treated as NO_FIX
	if ((0.0 == init_lat_deg) && (0.0 == init_lon_deg) && (0.0 == init_alt_m_hae))
	{
		latest_nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	}
	else
	{
		latest_nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
	}
	
	latest_nav_sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
	latest_nav_sat_fix.latitude = init_lat_deg;
	latest_nav_sat_fix.longitude = init_lon_deg;
	latest_nav_sat_fix.altitude = init_alt_m_hae;
	latest_nav_sat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
	
	latest_odometry.header.seq = 0;
	latest_odometry.header.frame_id = DEFAULT_EARTH_FIXED_FRAME_ID;
	latest_odometry.header.stamp = now;
	latest_odometry.child_frame_id = DEFAULT_ROBOT_FIXED_BASE_FRAME_ID;
	latest_odometry.pose.pose.position.x = init_position_x;
	latest_odometry.pose.pose.position.y = init_position_y;
	latest_odometry.pose.pose.position.z = init_position_z;
	latest_odometry.pose.pose.orientation.x = init_orientation_x;
	latest_odometry.pose.pose.orientation.y = init_orientation_y;
	latest_odometry.pose.pose.orientation.z = init_orientation_z;
	latest_odometry.pose.pose.orientation.w = init_orientation_w;
	// TODO: Configurable twist initialization?
	latest_odometry.twist = geometry_msgs::TwistWithCovariance(); // zero-initialized
	
	latest_heading.header.seq = 0;
	latest_heading.header.frame_id = DEFAULT_EARTH_FIXED_FRAME_ID;
	latest_heading.header.stamp = now;
	latest_heading.heading = init_heading_deg;
	
	setupStaticAHRSTransform();

	// And broadcast initial odom TF if so configured
	if (true == broadcast_odom_tf)
	{
		broadcastLatestOdomAsTF();
	}	
}


void NavPoseMgr::saveDataHandler(const nepi_ros_interfaces::SaveData::ConstPtr &msg)
{
		if ((msg->save_continuous == true) && (nullptr == data_fd))
		{
		    ROS_INFO("saveDataHandler recieved Save Data msg");
			checkForNewFile = false;
			ROS_INFO("saveDataHandler calling closeDataFile just in case");
			closeDataFile();
			ROS_INFO("saveDataHandler calling startNewDataFile");
			startNewDataFile();
		}
}

void NavPoseMgr::snapshotTriggerHandler(const std_msgs::Empty::ConstPtr &msg)
{
	bool saving_enabled = save_data_if->saveContinuousEnabled();
	if (false == saving_enabled)
	{
		    ROS_INFO("snapshotTriggerHandler received Snapshot Trigger msg");
			checkForNewFile = false;
			ROS_INFO("snapshotTriggerHandler calling closeDataFile");
			closeDataFile();
			ROS_INFO("snapshotTriggerHandler calling startNewDataFile");
			startNewDataFile();
			ROS_INFO("snapshotTriggerHandler calling closeDataFile");
			closeDataFile();
	}
	else
	{
		ROS_INFO("snapshotTriggerHandler ignoring message as Save Data is enabled");
	}
}


void NavPoseMgr::saveDataIfNecessary()
{	
	if (nullptr != data_fd)
	{
		const bool saving_enabled = save_data_if->saveContinuousEnabled();
		ros::Time data_time = ros::Time::now();
		const bool should_save = save_data_if->dataProductShouldSave("nav_pose",data_time);
		const bool snapshot_enabled = save_data_if->snapshotEnabled("nav_pose");
		if ((true == saving_enabled) && (true == should_save))
		{
			//ROS_INFO("Entered saveDataIfNecassary should save");
			if (nullptr != data_fd)
			{
				//ROS_INFO("Entered saveDataIfNecassary call saveNewData");
				saveNewData();  // New file opened by saveDataHandler
			}
		}
		else if ((false == saving_enabled) && (false == snapshot_enabled))
		{
			//ROS_INFO("saveDataIfNecassary calling closeDataFile(). No saving enabled");
			closeDataFile();
		}
	}	
	else
	{
		//ROS_INFO("saveDataIfNecassary skipping save. No file available");
	}
}



void NavPoseMgr::startNewDataFile()
{
	//ROS_INFO("Entered startNewDataFile");
	if (nullptr != data_fd)
	{
		//ROS_INFO("startNewDataFile closing existing file");
		closeDataFile();
	}
	// Build the filename
	const std::string display_name = _display_name;
	const std::string tstamp_str = save_data_if->getTimestampString();
	const std::string qualified_filename = save_data_if->getFullPathFilename(tstamp_str, display_name + "_nav", "yaml");
	const std::string save_prefix = save_data_if->getSavePrefixString();
	// Now create the file -- need to do this as a low-level open() call to set the permissions
	int fh = open(qualified_filename.c_str(), O_WRONLY | O_CREAT, 0664);
	if (fh < 0)
	{
		//ROS_ERROR("Unable to create file %s for saving nav/pos data", qualified_filename.c_str());
		return;
	}
	//ROS_INFO("startNewDataFile created new nav_pose data file %s", qualified_filename.c_str());

	// Now open it as a stream
	data_fd = fdopen(fh, "w");
	if (data_fd == nullptr)
	{
		//ROS_ERROR("Unable to open file stream for saving nav/pos data");
		return;
	}

	//LastSavePrefix = save_prefix;

	// Now print the header info
	// First, the current AHRS transform
	/*
	ros::Time latest(0.0);
	const std::string src_frame = latest_odometry.child_frame_id; // Seems backwards, but works
	const std::string target_frame = ahrs_out_frame_id; // Seems backwards, but works
	tf::StampedTransform transform;
	transform_listener.lookupTransform(target_frame, src_frame, latest, transform);
	geometry_msgs::TransformStamped transform_msg;
	tf::transformStampedTFToMsg(transform, transform_msg);

	*/
	//ROS_INFO("startNewDataFile saving new nav_pose data to file");
	fprintf(data_fd, "# Nav/Pose Data File\n");
	fprintf(data_fd, "start_time: %s\n", tstamp_str.c_str());
	fprintf(data_fd, "data:\n");

	/*
	fprintf(data_fd, "transform:\n");
	fprintf(data_fd, "  source_frame_id: %s\n", transform_msg.header.frame_id.c_str());
	//fprintf(data_fd, "  target_frame_id: %s\n", transform_msg.child_frame_id.c_str());
	fprintf(data_fd, "  # Translation in meters\n");
	fprintf(data_fd, "  translation:\n");
	fprintf(data_fd, "    x: %f\n", transform_msg.transform.translation.x);
	fprintf(data_fd, "    y: %f\n", transform_msg.transform.translation.y);
	fprintf(data_fd, "    z: %f\n", transform_msg.transform.translation.z);
	fprintf(data_fd, "  #Rotation Quaternion\n");
	fprintf(data_fd, "  rotation:\n");
	fprintf(data_fd, "    x: %f\n", transform_msg.transform.rotation.x);
	fprintf(data_fd, "    y: %f\n", transform_msg.transform.rotation.y);
	fprintf(data_fd, "    z: %f\n", transform_msg.transform.rotation.z);
	fprintf(data_fd, "    w: %f\n", transform_msg.transform.rotation.w);
	fprintf(data_fd, "entries:\n");
	*/

	writeDataToFile();
	fflush(data_fd);

}

void NavPoseMgr::closeDataFile()
{
	if (nullptr != data_fd)
	{
		ROS_INFO("Closing NavPose Data File");
		fclose(data_fd);
		data_fd = nullptr;
	}
}

void NavPoseMgr::saveNewData()
{   //ROS_INFO("Entered saveNewData");
	if (nullptr == data_fd)
	{   
		//ROS_INFO("saveNewData has no open file, Starting startNewDataFile");
		startNewDataFile();
	}
	else
	{
		bool new_save_triggered = save_data_if->newSaveTriggered();
		if ((true == checkForNewFile) && (true == new_save_triggered))
		{
			startNewDataFile();
			return;
		}
		//ROS_INFO("saveNewData has valid file");
		//ROS_INFO("saveNewData saving new data to file");
		writeDataToFile();
		fflush(data_fd);

		// Check if save file still needed
		const bool saving_enabled = save_data_if->saveContinuousEnabled();
		const bool snapshot_enabled = save_data_if->snapshotEnabled("nav_pose");
		if ((false == saving_enabled) && (false == snapshot_enabled))
		{
			//ROS_INFO("saveNewData starting closeDataFile");
			closeDataFile();
		}
		checkForNewFile = true; // start checking if prefix changed and new file needed
	}
}

void NavPoseMgr::writeDataToFile()
{
		//ROS_INFO("Entered writeDataToFile");
			nav_msgs::Odometry transformed_odom;
		/*
		if (false == save_data_if->saveRawEnabled())
		{
			// Implement Transform
			transformOdomData(latest_odometry, transformed_odom);
		}
		*/

		// Print the GPS fix here directly
		// GPS
		fprintf(data_fd, " \n");
		fprintf(data_fd, "     gps:\n");
		fprintf(data_fd, "       timestamp: %f\n", latest_nav_sat_fix.header.stamp.toSec());
		fprintf(data_fd, "       status: %s\n", navSatStatusToString(latest_nav_sat_fix.status.status));
		fprintf(data_fd, "       service: %s\n", navSatServiceToString(latest_nav_sat_fix.status.service));
		fprintf(data_fd, "       latitude: %f\n", latest_nav_sat_fix.latitude);
		fprintf(data_fd, "       longitude: %f\n", latest_nav_sat_fix.longitude);
		fprintf(data_fd, "       altitude: %f\n", latest_nav_sat_fix.altitude);
		
		// Odometry
		fprintf(data_fd, "     odometry:\n");
		fprintf(data_fd, "       timestamp: %f\n", latest_odometry.header.stamp.toSec());
		fprintf(data_fd, "       source_frame: %s\n", latest_odometry.header.frame_id.c_str());
		//fprintf(data_fd, "       output_frame: %s\n", transformed_odom.child_frame_id.c_str());
		fprintf(data_fd, "       position:\n");
		fprintf(data_fd, "          x: %f\n", latest_odometry.pose.pose.position.x);
		fprintf(data_fd, "          y: %f\n", latest_odometry.pose.pose.position.y);
		fprintf(data_fd, "          z: %f\n", latest_odometry.pose.pose.position.z);
		fprintf(data_fd, "       orientation:\n");
		fprintf(data_fd, "          quaternion_x: %f\n", latest_odometry.pose.pose.orientation.x);
		fprintf(data_fd, "          quaternion_y: %f\n", latest_odometry.pose.pose.orientation.y);
		fprintf(data_fd, "          quaternion_z: %f\n", latest_odometry.pose.pose.orientation.z);
		fprintf(data_fd, "          quaternion_w: %f\n", latest_odometry.pose.pose.orientation.w);
		fprintf(data_fd, "       linear_velocity:\n");
		fprintf(data_fd, "          x: %f\n", latest_odometry.twist.twist.linear.x);
		fprintf(data_fd, "          y: %f\n", latest_odometry.twist.twist.linear.y);
		fprintf(data_fd, "          z: %f\n", latest_odometry.twist.twist.linear.z);
		fprintf(data_fd, "       angular_velocity:\n");
		fprintf(data_fd, "          x: %f\n", latest_odometry.twist.twist.angular.x);
		fprintf(data_fd, "          y: %f\n", latest_odometry.twist.twist.angular.y);
		fprintf(data_fd, "          z: %f\n", latest_odometry.twist.twist.angular.z);
		
		// Heading 
		fprintf(data_fd, "     heading:\n");
		fprintf(data_fd, "       timestamp: %f\n", latest_heading.header.stamp.toSec());
		fprintf(data_fd, "       deg: %f\n", latest_heading.heading);
		fprintf(data_fd, "       ref: %s\n", (true == latest_heading.true_north)? "true" : "magnetic");

		// Reset snapshot enabled value
		save_data_if->snapshotReset("nav_pose");
}

} // namespace Numurus

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);

	Numurus::NavPoseMgr nav_pose_mgr;
	nav_pose_mgr.run();

	return 0;
}
