/*
 * Copyright (c) 2012, Stefano Rosa, Luca Carlone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <occupancy_grid_utils/ray_tracer.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <stdio.h>
#include<vector>

using namespace std;

namespace gm=geometry_msgs;
namespace sm=sensor_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::vector;
using std::string;

typedef boost::mutex::scoped_lock Lock;
typedef boost::shared_ptr<gu::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<gu::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;


/**
 * Class for occupancy grid map contruction
 * This class creates and publishes a map starting from
 * localized point clouds received by the graph_slam package
*/
class GridConstructionNode
{
public:
	GridConstructionNode ();

private:

	void scanCallback (const gu::LocalizedCloud::Ptr& loc_cloud);
	void buildGrid (const ros::WallTimerEvent& e);
	void buildGrid ();

	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	ros::Subscriber scan_sub_;
	ros::Publisher grid_pub_;
	ros::WallTimer build_grid_timer_;
	boost::mutex mutex_;
	tf::Transform base_to_laser_, laser_to_base_;
	CloudBuffer clouds_;
	CloudConstPtr last_cloud_;

	const unsigned history_length_;				// max number of localized point clouds to be used for mapo building
	const double resolution_;					// resolution of the map
	const string fixed_frame_;					// map frame
	const string sensor_frame_;					// laser frame
	const double grid_construction_interval_;	// time interval for map building (and publishing)
	const double local_grid_size_;				// map dimensions in (m)
	vector<geometry_msgs::Pose> trajectory;
	long count;

};


template <class T>
T getPrivateParam(const string& name, const T& default_value)
{
	ros::NodeHandle nh("~");
	T value;
	nh.param(name, value, default_value);
	return value;
}


GridConstructionNode::GridConstructionNode () :
		  history_length_(getPrivateParam("history_length", 5000)), resolution_(getPrivateParam("resolution", 0.05)),
		  fixed_frame_("world"), sensor_frame_("laser"),
		  grid_construction_interval_(getPrivateParam("grid_construction_interval", 0.3)),
		  local_grid_size_(getPrivateParam("local_grid_size", 100.0)),
		  scan_sub_(nh_.subscribe("loc_cloud", 500, &GridConstructionNode::scanCallback, this)),  //scan_sub_(nh_.subscribe("base_scan", 1, &GridConstructionNode::scanCallback, this)),
		  grid_pub_(nh_.advertise<nm::OccupancyGrid>("map", 1)),
		  build_grid_timer_(nh_.createWallTimer(ros::WallDuration(grid_construction_interval_),
				  &GridConstructionNode::buildGrid, this)),
				  clouds_(history_length_), count(0)
{
	ROS_INFO("Starting GridConstructionNode");
}


void createTfFromXYTheta(
		double x, double y, double theta, tf::Transform& t)
{
	t.setOrigin(btVector3(x, y, 0.0));
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, theta);
	t.setRotation(q);
}

double getYawFromQuaternion(
		const tf::Quaternion& quaternion)
{
	double temp, yaw;
	btMatrix3x3 m(quaternion);
	m.getRPY(temp, temp, yaw);
	return yaw;
}

double getYawFromQuaternion(
		const geometry_msgs::Quaternion& quaternion)
{
	tf::Quaternion q;
	tf::quaternionMsgToTF(quaternion, q);
	return getYawFromQuaternion(q);
}

/*
 * callback for localized point clouds
 */
void GridConstructionNode::scanCallback (const gu::LocalizedCloud::Ptr& loc_cloud)
{
	Lock lock(mutex_);
	last_cloud_=loc_cloud;

	if (last_cloud_)
	{
		if(last_cloud_->sensor_pose.position.z == -1.0)
		{
			ROS_INFO("Resetting map");
			clouds_.clear();
		}
		else
			clouds_.push_back(last_cloud_);

		last_cloud_.reset();
	}


}


/*
 * build and publish a grid map from stored localized point clouds
 */
void GridConstructionNode::buildGrid (const ros::WallTimerEvent& scan)
{
	if(clouds_.size()==0) return;
	ROS_DEBUG( "Building grid map with %d scans",clouds_.size());

	// Set up map dimensions
	nm::MapMetaData info;
	info.origin.position.x = -local_grid_size_/2; //odom_pose.pose.position.x-local_grid_size_/2;
	info.origin.position.y = -local_grid_size_/2; //odom_pose.pose.position.y-local_grid_size_/2;
	info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
	info.resolution = resolution_;
	info.width = local_grid_size_/resolution_;
	info.height = local_grid_size_/resolution_;

	nm::OccupancyGrid fake_grid;
	fake_grid.info = info;

	gu::OverlayClouds overlay = gu::createCloudOverlay(fake_grid, fixed_frame_, 0.3, 10, 2);

	vector<CloudConstPtr> clouds(clouds_.begin(), clouds_.end());
	BOOST_FOREACH  (CloudConstPtr cloud, clouds_)
	gu::addCloud(&overlay, cloud);
	nm::OccupancyGrid::ConstPtr grid = gu::getGrid(overlay);

	grid_pub_.publish(grid);

}



int main (int argc, char** argv)
{
  ros::init(argc, argv, "grid_mapper");
  GridConstructionNode node;
  ros::spin();
  return 0;
}
