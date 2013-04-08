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

/*  This package uses Canonical Scan Matcher [1], written by 
 *  Andrea Censi
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric" 
 *  Proceedings of the IEEE International Conference 
 *  on Robotics and Automation (ICRA), 2008
 */

#ifndef GRAPH_SLAM__H
#define GRAPH_SLAM__H

#include <vector>
#include <fstream>
#include <sstream>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <occupancy_grid_utils/LocalizedCloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <csm/csm_all.h>
#include <laser_geometry/laser_geometry.h>


#undef min 
#undef max


// structure defining a node in the graph
typedef struct {
	int id;
	LDP ldpScan;  // laser scan ( csm format)
	sensor_msgs::LaserScan scanMsg; // laser scan (ros format)
	geometry_msgs::Pose2D pose; // laser pose
	bool send;  //send node to mapper
} Node;

// structure defining an edge in the graph
typedef struct {
	int id1,id2;
	tf::Transform t; // transform
	Eigen::Matrix3d inf; // information matrix
	bool active; // used for disabling an edge
} Edge;

typedef enum {full_pose,orientation_only} edge_type;

typedef enum {lago,g2o,toro} optimizer_type;

// input topics
std::string scan_topic_  = "base_scan";
const std::string cloud_topic_ = "cloud";
std::string odom_topic_  = "odom";
const std::string imu_topic_   = "imu/data";

// output topics
const std::string pose_topic_ = "pose2D";
const std::string loc_cloud_topic_ = "loc_cloud";
// add traj topic...

// other variables
typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



/**
 * Main class for graph-slam
 * This class implements a laser-based front-end for graph-based slam optimization
*/
class GraphSlam
{

public:
	GraphSlam(ros::NodeHandle nh, ros::NodeHandle nh_private);
	~GraphSlam();

private:
	// **** graph creation variables
	long nodeCount;  	// # of nodes
	int nodeStep;    	// insert node every #nodeStep steps
	int currentIndex;	// index of current node
	std::vector<Node> nodes;	// list of nodes
	std::vector<Edge> edges;	// odometric edges
	std::vector<Edge> loops;	// loop closing edges
	int intraNodFails, totNodFails;  // icl failure counters
	int idleTime;  // avoids loop closings with preceding nodes
	int lastLongLoop;
	int OdomEdgesType; // use odometry for cartesian displacement
	int allowLoopClosings;

	// **** visualization variables
	geometry_msgs::PoseArray trajectory;
	geometry_msgs::PoseArray trajectory_map;

	visualization_msgs::MarkerArray loop_markers;

	// **** scan matching variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform base_to_laser_;
    tf::Transform laser_to_base_;

    ros::Publisher  pose_publisher_; // robot pose from csm
    ros::Publisher  vel_publisher_;  // not used
    ros::Publisher  loc_cloud_publisher_;  // for mapping
    ros::Publisher  trajectory_publisher_;  // trajectory for visualization
    ros::Publisher  trajectory_map_publisher_; // trajectory for mapping
    ros::Publisher  loops_publisher_;  // loops for visualization

    // **** other variables
    std::string base_frame_;
    std::string fixed_frame_;
    bool publish_tf_;
    bool publish_pose_;

    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /odom topic
    // 2) odom - [x, y, theta] from wheel odometry - /imu topic
    // 3) alpha_beta - [x, y, theta] from simple tracking filter - no topic req.
    // If more than one is enabled, priority is imu > odom > alpha_beta
    bool use_imu_;
    bool use_odom_;
    bool use_alpha_beta_;

    double alpha_;
    double beta_;

    // **** state variables
    bool initialized_;

    bool received_imu_;
    bool received_odom_;

    boost::mutex mutex_;

    tf::Transform last_w2b_;	// world-to-base tf of previous node
    tf::Transform w2b_; 		// current world-to-base tf (pose of base frame)

    double v_x_;  // velocity estimated by the alpha-beta tracker
    double v_y_;  
    double v_a_;

    ros::Time last_icp_time_;

    double latest_imu_yaw_;
    double last_imu_yaw_;

    geometry_msgs::Quaternion latest_imu_q_;
    geometry_msgs::Quaternion last_imu_q_;

    nav_msgs::Odometry latest_odom_;
    nav_msgs::Odometry last_odom_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    std::string vertex_name;
    std::string edge_name;
    std::string optimizer_name;

    int optimizerType;	// optimizer to be used, changes input/output file format as well

    // **** methods

    // process a new laser scan, insert nodes, odometric edges and loop closings
    //void processScan(LDP& curr_ldp_scan, const sensor_msgs::LaserScan::ConstPtr& scan_msg, const ros::Time& time);
    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
    void imuCallback (const sensor_msgs::ImuPtr& imu_msg);

    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const std::string& frame_id);
    void initParams();

    void getPrediction(double& pr_ch_x, double& pr_ch_y, 
                       double& pr_ch_a, double dt);

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    double getYawFromQuaternion(const tf::Quaternion& quaternion);
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);

    // find a possible loop between two scans
    tf::Transform findLoop(LDP& scan1, LDP& scan2, geometry_msgs::Pose2D prediction, double * icpError, Eigen::Matrix3d *inf);
    // do graph optimization and update trajectory
    void optimize();
    // update nodes after doing optimization
    void updateNodes();
    // check trajectory consistency
    bool checkNodes();
    // check visibility of node b from node a
    bool checkVisibility(Node a, Node b);


    bool checkEdges();
    double calcVectorAvg(std::vector<double> vec);
    geometry_msgs::Pose2D calcPoseDiff2D(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b);
    double calcCartMismatch(tf::Transform a, tf::Transform b );
    double calcAngMismatch(tf::Transform a, tf::Transform b );
    void resetMap();
    // send new trajectory
    void updateMap();

    double normalizeAngle(double angle);

};


#endif // GRAPH_SLAM__H
