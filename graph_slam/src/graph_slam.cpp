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

#include "graph_slam/graph_slam.h"

using namespace std;


/**
 * Main constructor
 */
GraphSlam::GraphSlam(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting GraphSlam Optimizer");

  // **** init parameters
  initParams();

  switch(optimizerType)
  {
	  case lago:
		  vertex_name= "VERTEX2";
		  edge_name="EDGE2";
		  optimizer_name ="graphSLAM_sc";
		  break;
	  case toro:
		  vertex_name= "VERTEX2";
		  edge_name="EDGE2";
		  optimizer_name ="toro";
		  break;
	  case g2o:  // with vertigo library
		  vertex_name= "VERTEX_SE2";
		  edge_name= "EDGE_SE2";
		  optimizer_name = "g2o -typeslib /home/p3dx/slam/vertigo/trunk/lib/libvertigo-g2o.so -o output_graph.txt";
		  break;
  }

  // **** state variables 
  initialized_   = false;
  received_imu_  = false;
  received_odom_ = false;

  w2b_.setIdentity();

  v_x_ = 0;
  v_y_ = 0;
  v_a_ = 0;

  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0; 
  input_.laser[2] = 0.0; 

  nodeCount=0;
  currentIndex=0;
  intraNodFails=0;
  totNodFails=0;
  idleTime=20;

  trajectory.header.frame_id=fixed_frame_;
  trajectory_map.header.frame_id=fixed_frame_;

  // *** subscribers
    scan_subscriber_ = nh_.subscribe(
      scan_topic_, 1, &GraphSlam::scanCallback, this);


  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      imu_topic_, 1, &GraphSlam::imuCallback, this);
  }

  if (use_odom_)
  {
    odom_subscriber_ = nh_.subscribe(
      odom_topic_, 1, &GraphSlam::odomCallback, this);
  }

  // **** publishers
  if (publish_pose_)
  {
    pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
      pose_topic_, 5);
  }

  loc_cloud_publisher_= nh_.advertise<occupancy_grid_utils::LocalizedCloud>(
	  loc_cloud_topic_ , 5);

  trajectory_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
		  "trajectory" , 5);

  trajectory_map_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
		  "trajectory_map" , 5);

  loops_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
		  "loops" , 5);

}

GraphSlam::~GraphSlam()
{

}

/**
 * Parse parameters
 */
void GraphSlam::initParams()
{
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";

  if (!nh_private_.getParam ("scan_topic_", scan_topic_))
	  scan_topic_ = "base_scan";
  if (!nh_private_.getParam ("odom_topic_", odom_topic_))
	  odom_topic_ = "odom";

  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = false;
  if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = true;
  if (!nh_private_.getParam ("use_alpha_beta", use_alpha_beta_))
    use_alpha_beta_ = false;


  // publish pose message (pose of base frame in the fixed frame)
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;

  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 1.0;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.8;

 // CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
	// always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
	//  1) Order the errors.
	//	2) Choose the percentile according to outliers_adaptive_order.
	//	   (if it is 0.7, get the 70% percentile)
	//	3) Define an adaptive threshold multiplying outliers_adaptive_mult
	//	   with the value of the error at the chosen percentile.
	//	4) Discard correspondences over the threshold.
	//	This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  //If you already have a guess of the solution, you can compute the polar angle
	//	of the points of one scan in the new position. If the polar angle is not a monotone
	//	function of the readings index, it means that the surface is not visible in the 
	//	next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the 
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the 
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;



  // front-end parameters

  // insert a corrected pose and the relative scan into trajectory and
  // check for loop closures every nodeStep steps
  if (!nh_private_.getParam ("nodeStep", nodeStep))
	  nodeStep = 10;

  // This flag decides if to use the odometric or the scan-matcher cartesian displacement
  // for the odometric edges
  if (!nh_private_.getParam ("OdomEdgesType", OdomEdgesType))
	  OdomEdgesType = 0;

  // This flag decides if the algorithm should perform SLAM optimization
  if (!nh_private_.getParam ("allowLoopClosings", allowLoopClosings))
	  allowLoopClosings = 1;



  // back-end parameters

  // This integer decides the graph optimizer to be used
  if (!nh_private_.getParam ("optimizerType", optimizerType))
	  optimizerType = lago;
}


/**
 * Callback for IMU sensor data (if present)
 */
void GraphSlam::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_imu_yaw_ = getYawFromQuaternion(imu_msg->orientation);
  latest_imu_q_ = imu_msg->orientation;
  if (!received_imu_)
  {
    last_imu_yaw_ = getYawFromQuaternion(imu_msg->orientation);
    last_imu_q_ = imu_msg->orientation;
    received_imu_ = true;
  }
}

/**
 * Callback for odometry data
 */
void GraphSlam::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);
  latest_odom_ = *odom_msg;
  if (!received_odom_)
  {
    last_odom_ = *odom_msg;
    received_odom_ = true;
    lastLongLoop=0;
  }
}

/**
 * Callback for laser data
 */
void GraphSlam::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner
  
  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("ScanMatcher: Skipping scan");
      return;
    }

    laserScanToLDP(scan_msg, prev_ldp_scan_); 
    last_icp_time_ = scan_msg->header.stamp;
    last_imu_yaw_ = latest_imu_yaw_;
    last_imu_q_ = latest_imu_q_;
    last_odom_ = latest_odom_;
    initialized_ = true;
  }


  processScan(scan_msg);
}


/**
 * Process the current laser scan, create a new graph node if needed, search for loop closings
 *
 * @param scan_msg current laser scan message
 */
void GraphSlam::processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  ros::Time time=	scan_msg->header.stamp;
  LDP curr_ldp_scan;
  laserScanToLDP(scan_msg, curr_ldp_scan);
  struct timeval start, end;    // used for timing
  gettimeofday(&start, NULL);

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan 
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0;
  prev_ldp_scan_->odometry[1] = 0;
  prev_ldp_scan_->odometry[2] = 0;

  prev_ldp_scan_->estimate[0] = 0;
  prev_ldp_scan_->estimate[1] = 0;
  prev_ldp_scan_->estimate[2] = 0;

  prev_ldp_scan_->true_pose[0] = 0;
  prev_ldp_scan_->true_pose[1] = 0;
  prev_ldp_scan_->true_pose[2] = 0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // estimated change since last scan

  ros::Time new_icp_time = ros::Time::now();
  ros::Duration dur = new_icp_time - last_icp_time_;
  double dt = dur.toSec();

  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  // the predicted change of the laser's position, in the base frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * pr_ch * base_to_laser_;
  
  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = getYawFromQuaternion(pr_ch_l.getRotation());

  ROS_DEBUG("OdomEdgesType = %d, %f, %f, %f\n", OdomEdgesType, input_.first_guess[0],
                         input_.first_guess[1], 
                         input_.first_guess[2]);

  // *** scan match - using point to line icp from CSM

  input_.do_compute_covariance = 1;

  sm_icp(&input_, &output_);

  if (output_.valid) 
  {
    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;

    // depending on the flag the odometric edges are built from sm, wheel,
    // a combination of wheels and sm, or a combination of sm and IMU
    // use wheel odometry for Cartesian part, and orientations from IMU
    // use output of scan matching

    switch(OdomEdgesType)
    {
		case 0:	createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l); // use wheel odometry for cartesian part and angle from CSM
				break;
		case 1: createTfFromXYTheta(pr_ch_l.getOrigin().getX(), pr_ch_l.getOrigin().getY(), output_.x[2], corr_ch_l); // use wheel odometry and bypass CSM
				break;
		case 2: createTfFromXYTheta(pr_ch_l.getOrigin().getX(), pr_ch_l.getOrigin().getY(), getYawFromQuaternion(pr_ch_l.getRotation()), corr_ch_l);
    }

    // the correction of the base's position, in the world frame
    tf::Transform corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;

    if (use_imu_ && received_imu_)
    {
    	 ROS_DEBUG("substituting orientations from IMU");
    	 corr_ch.setRotation( pr_ch_l.getRotation()  );
    }

    w2b_ = w2b_ * corr_ch;


    // publish robot pose

    if (publish_pose_) 
    {
      geometry_msgs::Pose2D::Ptr pose_msg;
      pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
      pose_msg->x = w2b_.getOrigin().getX();
      pose_msg->y = w2b_.getOrigin().getY();
      pose_msg->theta = getYawFromQuaternion(w2b_.getRotation());
      pose_publisher_.publish(pose_msg);
    }
    if (publish_tf_)
    {
      tf::StampedTransform transform_msg (w2b_, time, fixed_frame_, base_frame_);
      tf_broadcaster_.sendTransform (transform_msg);
    }


    // every nodeStep steps update graph and search for loop closings
    if( ( nodeCount % nodeStep == 0 && (calcCartMismatch(last_w2b_,w2b_)>0.01 || calcAngMismatch(last_w2b_,w2b_)>0.01))
    	|| nodeCount==0)

    {

    	if(currentIndex==0)
    		resetMap();

		// publish *current* localized point cloud (laser scan + pose) for mapping
		sensor_msgs::PointCloud fixed_frame_cloud;
		laser_geometry::LaserProjection projector_;
		projector_.projectLaser(*scan_msg, fixed_frame_cloud);
		occupancy_grid_utils::LocalizedCloud loc_cloud;
		loc_cloud.cloud.points = fixed_frame_cloud.points;
		geometry_msgs::Pose pose;

		tf::Transform w2l_ = w2b_ * base_to_laser_;
		pose.position.x=w2l_.getOrigin().getX();
		pose.position.y=w2l_.getOrigin().getY();
		pose.position.z =w2l_.getOrigin().getZ();
		pose.orientation.x = w2l_.getRotation().getX();
		pose.orientation.y = w2l_.getRotation().getY();
		pose.orientation.z = w2l_.getRotation().getZ();
		pose.orientation.w = w2l_.getRotation().getW();
		loc_cloud.sensor_pose = pose;
		loc_cloud.header.frame_id = fixed_frame_;

		// publish whole trajectory
		trajectory.poses.push_back(pose);
		trajectory_publisher_.publish(trajectory);


		// create new node
		Node node;
		node.scanMsg = *scan_msg;
		LDP tmp_ldp_scan;
		laserScanToLDP(scan_msg, tmp_ldp_scan);
		node.ldpScan = tmp_ldp_scan;
		node.id = currentIndex;
		node.pose.x=w2b_.getOrigin().getX();
		node.pose.y=w2b_.getOrigin().getY();
		node.pose.theta= getYawFromQuaternion(w2b_.getRotation());
		node.send = true;



		// do not send
		for(unsigned i = 0; i < nodes.size(); i++)
		if(nodes[i].send==true)
		{
			tf::Transform tf_i, tf_j;
			createTfFromXYTheta(nodes[i].pose.x, nodes[i].pose.y, nodes[i].pose.theta, tf_i);
			createTfFromXYTheta(node.pose.x, node.pose.y, node.pose.theta, tf_j);

				if( calcCartMismatch(tf_i, tf_j) < 0.5 && calcAngMismatch(tf_i, tf_j) < 0.2 )
				//if( calcCartMismatch(tf_i, tf_j) < 0.1 && calcAngMismatch(tf_i, tf_j) < 0.1 )
				{
					ROS_DEBUG("Not sending node, distance: %f %f", calcCartMismatch(tf_i, tf_j),calcAngMismatch(tf_i, tf_j) );
					node.send = false;
					break;
				}

			}
		if (node.send)
		{
			trajectory_map.poses.push_back(pose);
			loc_cloud_publisher_.publish(loc_cloud);
		}

		// publish poses used for mapping
		trajectory_map_publisher_.publish(trajectory_map);

		nodes.push_back(node);

		// create odometric constraint
		if(currentIndex > 0)
		{
			Eigen::Matrix3d cov;
			// store covariance matrix
			///////////////////////////////////////////////////////
			if( (output_.cov_x_m->size1==3 && output_.cov_x_m->size2==3) || intraNodFails > 0 )
			    {
				   cov(0,0) = output_.cov_x_m->data[0];
				   cov(0,1) = output_.cov_x_m->data[1];
				   cov(0,2) = output_.cov_x_m->data[2];
				   cov(1,0) = output_.cov_x_m->data[3];
				   cov(1,1) = output_.cov_x_m->data[4];
				   cov(1,2) = output_.cov_x_m->data[5];
				   cov(2,0) = output_.cov_x_m->data[6];
				   cov(2,1) = output_.cov_x_m->data[7];
				   cov(2,2) = output_.cov_x_m->data[8];
			    }
			else
				{
				   cov(0,0) = 0.5*0.5;
				   cov(0,1) = 0.0;
				   cov(0,2) = 0.0;
				   cov(1,0) = 0.0;
				   cov(1,1) = 0.5*0.5;
				   cov(1,2) = 0.0;
				   cov(2,0) = 0.0;
				   cov(2,1) = 0.0;
				   cov(2,2) = 0.08*0.08;
				}
		    // cov = cov * nodeStep;  // we have nodeStep odometric steps between a pair of nodes

			 Eigen::Matrix3d inf = cov.inverse();
			///////////////////////////////////////////////////////

			tf::Transform loc_transform= last_w2b_.inverse()*w2b_;
			Edge edge;
			edge.id1 = currentIndex-1;
			edge.id2 = currentIndex;
			edge.t=loc_transform;
			edge.inf = inf;
			edges.push_back(edge);
		}


		// search for loops
		bool loopFound = false;
		bool flag_ang_loop = false;
		int discardedForPrediction=0;
		int discardedForInnovation=0;
		int discaredForICPError=0;
		int acceptedLoops=0; // loops found at current step

		if(allowLoopClosings)
		for(int i = 0; i < (int)nodes.size() - 1; i++)
		{
			  // create expected relative pose between current node and i-th node
			  geometry_msgs::Pose2D prediction;
			  geometry_msgs::Pose2D pose;  // current pose
			  double x,y;
		      pose.x = w2b_.getOrigin().getX();
		      pose.y = w2b_.getOrigin().getY();
		      pose.theta = getYawFromQuaternion(w2b_.getRotation());
			  x= nodes[i].pose.x - pose.x;
			  y= nodes[i].pose.y - pose.y;
			  prediction.theta= nodes[i].pose.theta - pose.theta;
			  // normalize prediction angle (-pi,pi)
			  prediction.theta = normalizeAngle(prediction.theta);
			  // transformation in laser frame
			  prediction.x = cos(pose.theta)*x + sin(pose.theta)*y;
			  prediction.y = -sin(pose.theta)*x + cos(pose.theta)*y;


			  // if prediction > half of laser's max range we skip loop closing check
			  if(sqrt( prediction.x*prediction.x + prediction.y*prediction.y) > 4.0) //
			  {
				  discardedForPrediction++;
				  continue;
			  }

//			  if (!checkVisibility(node,nodes[i]) || !checkVisibility(nodes[i],node))
//			  {
//				  ROS_WARN("checkVisibility(): loop discarded because of visibility check!");
//				  continue;
//			  }

			  double icpError;
			  Eigen::Matrix3d inf;
			  // create expected rel pose transform
			  tf::Transform expected_ch;

			  int edgeType=full_pose;

			  createTfFromXYTheta(prediction.x, prediction.y, prediction.theta, expected_ch);

			  // if no loop is found returned transform is 0,0,0
			  tf::Transform loopConstraint = findLoop( curr_ldp_scan, nodes[i].ldpScan, prediction, &icpError, &inf);

			  //////////////////////////////////////////////////////////////////////////////////////////////
			  if (fabs(prediction.theta) > 0.5) // nodes are not alligned, hence the relative
				  //position measurement may be unreliable
			  {
				  edgeType=orientation_only;

				  inf(0,0)=0;
				  inf(1,0)=0;
				  inf(0,1)=0;
				  inf(1,1)=0;
				  inf(2,0)=0;
				  inf(0,2)=0;
				  inf(2,1)=0;
				  inf(1,2)=0;
			  }

			  if (i > (int)nodes.size() - idleTime) // nodes are too close, hence in corridors the
				  //position measurement may be unreliable
			  {
				  edgeType=orientation_only;

				  inf(0,0)=0;
				  inf(1,0)=0;
				  inf(0,1)=0;
				  inf(1,1)=0;
				  inf(2,0)=0;
				  inf(0,2)=0;
				  inf(2,1)=0;
				  inf(1,2)=0;
			  }
			  ////////////////////////////////////////////////////////////////////////////////////////////////7

			  if(icpError <= 0.1 || icpError >= 0.3)
			  {
				  ROS_DEBUG("ICP error: %f",icpError);
				  discaredForICPError++;
			  }
			  if(calcCartMismatch(expected_ch,loopConstraint) > 2.00 || calcAngMismatch(expected_ch,loopConstraint) > 0.05)
				  discardedForInnovation++;

			  int minSizeLongLoop = 100;
			  double maxCartMismatch = 2.0 + 5.0* min( (currentIndex-lastLongLoop) / minSizeLongLoop, 1 );
			  double maxAngMismatch = 0.05 + 0.25 * min( (currentIndex-lastLongLoop) / minSizeLongLoop, 1);

			  ROS_DEBUG("maxCartMismatch: %f, maxAngMismatch: %f", maxCartMismatch, maxAngMismatch);

			  if(loopConstraint.getOrigin().getX() != 0.0 && loopConstraint.getOrigin().getY() != 0.0 &&
					  getYawFromQuaternion(loopConstraint.getRotation()) != 0.0 &&
					  icpError > 0.1 && icpError < 0.5 && //0.1, 0.3
					  calcCartMismatch(expected_ch,loopConstraint) < maxCartMismatch &&
					  calcAngMismatch(expected_ch,loopConstraint) < maxAngMismatch  )
			  {// if all checks are satisfied then include the loop closing


				  if ((edgeType==full_pose) || ((edgeType==orientation_only) && flag_ang_loop==false))  // once we found an orientation loop we break, i.e., we do not add further loops
				  {
					  Edge loop;
					  loop.id1 = currentIndex;
					  loop.id2 = i;
					  loop.t.setOrigin(loopConstraint.getOrigin());
					  loop.t.setRotation(loopConstraint.getRotation());
					  loop.inf=inf;
					  loops.push_back(loop);
					  loopFound = true;

					  acceptedLoops++;
					  if( currentIndex - i > minSizeLongLoop )
						  lastLongLoop= currentIndex;

					  // send loop marker for visualization
					  geometry_msgs::Point p1,p2;
					  p1.x=w2b_.getOrigin().getX();
					  p1.y=w2b_.getOrigin().getY();
					  p1.z=0.0;
					  p2.x=nodes[i].pose.x;
					  p2.y=nodes[i].pose.y;
					  p2.z=0.0;
					  visualization_msgs::Marker loop_marker;
					  //loop_marker.header.frame = fixed_frame_;
					  loop_marker.type = 0;
					  loop_marker.action = visualization_msgs::Marker::ADD;
					  loop_marker.points.push_back(p1);
					  loop_marker.points.push_back(p2);
					  loop_marker.header.frame_id=fixed_frame_;

					  if(edgeType == orientation_only)
					  {
						  loop_marker.color.r = 1.0;
						  loop_marker.color.g = 1.0;
						  loop_marker.color.b = 0.0;
						  flag_ang_loop = true;
					  }
					  else
					  {
						  loop_marker.color.r = 0.0;
						  loop_marker.color.g = 1.0;
						  loop_marker.color.b = 0.0;
					  }

					  loop_marker.color.a = 1.0;
					  loop_marker.scale.x=0.05;
					  loop_marker.scale.y=0.2;
					  //loop_marker.scale.z=0.1;
					  loop_marker.lifetime = ros::Duration();
					  loop_marker.id=loop_markers.markers.size();
					  loop_marker.ns="loops";
					  loop_markers.markers.push_back(loop_marker);
				  }
			  }

		}
		//loop_markers.header.frame_id=fixed_frame_;
		loops_publisher_.publish(loop_markers);

		ROS_DEBUG("Loops tested nodes: %d; discarded for prediction: %d, icp error: %d, innovation: %d; accepted: %d", nodes.size()- idleTime,
				discardedForPrediction, discaredForICPError, discardedForInnovation, acceptedLoops);

		if( loopFound && allowLoopClosings==1)
		{
			optimize();
			updateNodes();

			//check updated trajectory for strange displacements
			if(nodes.size()>0  && (!checkNodes() || !checkEdges()) )
			{
				// delete last loops and redo optimization
				loops.erase(loops.end()-acceptedLoops, loops.end());
				loop_markers.markers.erase(loop_markers.markers.end()-acceptedLoops, loop_markers.markers.end());
				optimize();
				updateNodes();
			}
			// update w2b_
			createTfFromXYTheta(nodes[currentIndex].pose.x,  nodes[currentIndex].pose.y, nodes[currentIndex].pose.theta, w2b_);
			// reset and update map
			resetMap();
			updateMap();
		}


		// update pose of last node
		last_w2b_ = w2b_;

		//update current node id
		currentIndex++;

		intraNodFails = 0;
    }
  }
  else
  {
	w2b_ = w2b_ * pr_ch; // if csm fails we use the odometry information
    ROS_WARN("Error in scan matching");

    v_x_ = 0.0;
    v_y_ = 0.0;
    v_a_ = 0.0;

    intraNodFails++;
    totNodFails++;
  }

  // **** increase indexes
  nodeCount++;  // maybe duplicated by currentIndex
  // **** swap old and new

  ld_free(prev_ldp_scan_);
  prev_ldp_scan_ = curr_ldp_scan;
  last_icp_time_ = new_icp_time;

  // **** statistics

  gettimeofday(&end, NULL);
  double dur_total = ((end.tv_sec   * 1000000 + end.tv_usec  ) -
                      (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;
  ROS_DEBUG("scan matcher total duration: %.1f ms", dur_total);
}


// convert LaserScan message to LDP object for CSM
void GraphSlam::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      // fill in laser scan data  

      ldp->valid[i] = 1;
      ldp->readings[i] = r;   
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];
 
  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void GraphSlam::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear(); 
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}

bool GraphSlam::getBaseToLaserTf (const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, frame_id, t, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcher: Could not get initial laser transform(%s)", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

// returns the predicted change in pose (in fixed frame)
// since the last time ICP was done
void GraphSlam::getPrediction(double& pr_ch_x, double& pr_ch_y, double& pr_ch_a, double dt)
{
  boost::mutex::scoped_lock(mutex_);
  tf::Transform tf_latest;
  tf::Transform tf_last;

  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use alpha-beta tracking (const. vel. model)
  if (use_alpha_beta_)
  {
    // estmate change in fixed frame, using fixed velocity
    pr_ch_x = v_x_ * dt;     // in fixed frame
    pr_ch_y = v_y_ * dt;
    pr_ch_a = v_a_ * dt;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_)
  {

    pr_ch_x = latest_odom_.pose.pose.position.x - 
              last_odom_.pose.pose.position.x;

    pr_ch_y = latest_odom_.pose.pose.position.y - 
              last_odom_.pose.pose.position.y;

    pr_ch_a = getYawFromQuaternion(latest_odom_.pose.pose.orientation) -
              getYawFromQuaternion(last_odom_.pose.pose.orientation);
    pr_ch_a = normalizeAngle(pr_ch_a);


    double theta = getYawFromQuaternion(last_odom_.pose.pose.orientation);
    double temp[2];
    temp[0] =    cos(theta)*pr_ch_x + sin(theta)*pr_ch_y;
    temp[1] =   - sin(theta)*pr_ch_x + cos(theta)*pr_ch_y;
    pr_ch_x= temp[0];
    pr_ch_y= temp[1];


	createTfFromXYTheta(latest_odom_.pose.pose.position.x, latest_odom_.pose.pose.position.y, getYawFromQuaternion(latest_odom_.pose.pose.orientation), tf_latest);
	createTfFromXYTheta(last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y, getYawFromQuaternion(last_odom_.pose.pose.orientation), tf_last);

    last_odom_ = latest_odom_;
  }

  // **** use imu
  if (use_imu_ && received_imu_) // && pr_ch_a>0.01)
  {
	ROS_INFO("+++++++++++++++++++++++ Using the orientation from IMU +++++++++++++++++++++++++++");
	// conversion from geometry_msgs::Quaternion to tf::Quaternion
	tf::Quaternion last_q_(last_imu_q_.x,last_imu_q_.y,last_imu_q_.z,last_imu_q_.w);
	tf::Quaternion latest_q_(latest_imu_q_.x,latest_imu_q_.y,latest_imu_q_.z,latest_imu_q_.w);

    tf::Quaternion pr_ch_q = last_q_.inverse() * latest_q_;
    //pr_ch_a = latest_imu_yaw_ - last_imu_yaw_;

    ROS_INFO("pr_ch_a_odom: %f", pr_ch_a);

    if( (calcCartMismatch(tf_latest,tf_last) > 0.01 || calcAngMismatch(tf_latest,tf_last)>0.0001) )
    {
    pr_ch_a = getYawFromQuaternion(pr_ch_q);
    ROS_INFO("pr_ch_a_imu: %f", pr_ch_a);
    }
    else
    	ROS_INFO("small angle, using odometric rotation");
  }
  last_imu_q_ = latest_imu_q_;
  last_imu_yaw_ = latest_imu_yaw_;
}

double GraphSlam::getYawFromQuaternion(
  const tf::Quaternion& quaternion)
{
  double temp, yaw;
  btMatrix3x3 m(quaternion);
  m.getRPY(temp, temp, yaw);
  return yaw;
}

double GraphSlam::getYawFromQuaternion(
  const geometry_msgs::Quaternion& quaternion)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(quaternion, q);
  return getYawFromQuaternion(q);
}

void GraphSlam::createTfFromXYTheta(
  double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(btVector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

/*
 * detect possible loop closing between scan1 and scan2
 */
tf::Transform GraphSlam::findLoop(LDP& scan1, LDP& scan2, geometry_msgs::Pose2D prediction, double * icpError, Eigen::Matrix3d *inf)
{
  //geometry_msgs::Pose2D result;

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0;
  prev_ldp_scan_->odometry[1] = 0;
  prev_ldp_scan_->odometry[2] = 0;

  prev_ldp_scan_->estimate[0] = 0;
  prev_ldp_scan_->estimate[1] = 0;
  prev_ldp_scan_->estimate[2] = 0;

  prev_ldp_scan_->true_pose[0] = 0;
  prev_ldp_scan_->true_pose[1] = 0;
  prev_ldp_scan_->true_pose[2] = 0;

  input_.laser_ref  = scan1;
  input_.laser_sens = scan2;

  // **** estimated change since last scan

  double pr_ch_x = prediction.x;
  double pr_ch_y = prediction.y;
  double pr_ch_a = prediction.theta;

  // the predicted change of the laser's position, in the base frame

  tf::Transform pr_ch;
  createTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, pr_ch);

  // the predicted change of the laser's position, in the laser frame

  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * pr_ch * base_to_laser_;

  input_.first_guess[0] = pr_ch_l.getOrigin().getX();
  input_.first_guess[1] = pr_ch_l.getOrigin().getY();
  input_.first_guess[2] = getYawFromQuaternion(pr_ch_l.getRotation());

  input_.do_compute_covariance = 1;

   // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);

  *icpError=1000000.0;
  tf::Transform corr_ch_l;

  if (output_.valid)
  {
	*icpError=output_.error;
    // the correction of the laser's position, in the laser frame
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    corr_ch_l = base_to_laser_ * corr_ch_l * laser_to_base_;

    Eigen::Matrix3d cov;
	// store covariance matrix
	///////////////////////////////////////////////////////
	if(output_.cov_x_m->size1==3 && output_.cov_x_m->size2==3)
		{
		   cov(0,0) = output_.cov_x_m->data[0];
		   cov(0,1) = output_.cov_x_m->data[1];
		   cov(0,2) = output_.cov_x_m->data[2];
		   cov(1,0) = output_.cov_x_m->data[3];
		   cov(1,1) = output_.cov_x_m->data[4];
		   cov(1,2) = output_.cov_x_m->data[5];
		   cov(2,0) = output_.cov_x_m->data[6];
		   cov(2,1) = output_.cov_x_m->data[7];
		   cov(2,2) = output_.cov_x_m->data[8];

		}
	else
		{
		   cov(0,0) = 0.5*0.5;
		   cov(0,1) = 0.0;
		   cov(0,2) = 0.0;
		   cov(1,0) = 0.0;
		   cov(1,1) = 0.5*0.5;
		   cov(1,2) = 0.0;
		   cov(2,0) = 0.0;
		   cov(2,1) = 0.0;
		   cov(2,2) = 0.08*0.08;
		}
	 *inf = cov.inverse();
	///////////////////////////////////////////////////////
  }
  else
	  ROS_WARN("ERROR: laser input not valid!");


  return corr_ch_l;
}

/*
 * write input graph file and call the optimizer
 */
void GraphSlam::optimize()
{
	FILE *out=fopen("input.graph","w");

	// for debug
	FILE *nodes_csv=fopen("nodes.csv","w");
	FILE *edges_csv=fopen("edges.csv","w");
	FILE *loops_csv=fopen("loops.csv","w");

	for( unsigned i=0; i < nodes.size(); i++)
	{
		fprintf(out,"%s %d %f %f %f\n",vertex_name.c_str(), nodes[i].id, nodes[i].pose.x, nodes[i].pose.y, nodes[i].pose.theta);
		fprintf(nodes_csv,"%d,%f,%f,%f\n", nodes[i].id, nodes[i].pose.x, nodes[i].pose.y, nodes[i].pose.theta);

	}
	for( unsigned i=0; i < edges.size(); i++)
	{
		if(optimizerType==lago || optimizerType==toro)
			fprintf(out,"%s %d %d %f %f %f %f %f %f %f %f %f\n",edge_name.c_str(), edges[i].id1, edges[i].id2, edges[i].t.getOrigin().getX(), edges[i].t.getOrigin().getY(),
					getYawFromQuaternion(edges[i].t.getRotation()),
					edges[i].inf(0,0), edges[i].inf(0,1), edges[i].inf(1,1), edges[i].inf(2,2), edges[i].inf(0,2), edges[i].inf(1,2));
		if(optimizerType==g2o)
			fprintf(out,"%s %d %d %f %f %f %f %f %f %f %f %f\n",edge_name.c_str(), edges[i].id1, edges[i].id2, edges[i].t.getOrigin().getX(), edges[i].t.getOrigin().getY(),
							getYawFromQuaternion(edges[i].t.getRotation()),
							edges[i].inf(0,0), edges[i].inf(0,1), edges[i].inf(0,2), edges[i].inf(1,1), edges[i].inf(1,2), edges[i].inf(2,2));
		fprintf(edges_csv,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", edges[i].id1, edges[i].id2, edges[i].t.getOrigin().getX(), edges[i].t.getOrigin().getY(),
				getYawFromQuaternion(edges[i].t.getRotation()),
				edges[i].inf(0,0), edges[i].inf(0,1), edges[i].inf(1,1), edges[i].inf(2,2), edges[i].inf(0,2), edges[i].inf(1,2));
	}
	for( unsigned i=0; i < loops.size(); i++)
	{
		if(optimizerType==lago || optimizerType==toro)
			fprintf(out,"%s %d %d %f %f %f %f %f %f %f %f %f\n",edge_name.c_str(), loops[i].id1, loops[i].id2, loops[i].t.getOrigin().getX(), loops[i].t.getOrigin().getY(),
					getYawFromQuaternion(loops[i].t.getRotation()),
					loops[i].inf(0,0), loops[i].inf(0,1), loops[i].inf(1,1), loops[i].inf(2,2), loops[i].inf(0,2), loops[i].inf(1,2));
		if(optimizerType==g2o)
		{
			int newEdge=nodes.size() + i;
			fprintf(out,"VERTEX_SWITCH %d 1\n",newEdge);
			fprintf(out,"EDGE_SWITCH_PRIOR %d 1 1.0\n",newEdge);
			fprintf(out,"EDGE_SE2_SWITCHABLE %d %d %f %f %f %f %f %f %f %f %f\n", loops[i].id1, loops[i].id2, loops[i].t.getOrigin().getX(), loops[i].t.getOrigin().getY(),
								getYawFromQuaternion(loops[i].t.getRotation()),
								loops[i].inf(0,0), loops[i].inf(0,1), loops[i].inf(0,2), loops[i].inf(1,1), loops[i].inf(1,2), loops[i].inf(2,2));
		}

		fprintf(loops_csv,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", loops[i].id1, loops[i].id2, loops[i].t.getOrigin().getX(), loops[i].t.getOrigin().getY(),
				getYawFromQuaternion(loops[i].t.getRotation()),
				loops[i].inf(0,0), loops[i].inf(0,1), loops[i].inf(1,1), loops[i].inf(2,2), loops[i].inf(0,2), loops[i].inf(1,2));
	}
	fclose(nodes_csv);
	fclose(edges_csv);
	fclose(loops_csv);

	fclose(out);
	// the horror
	char cmd[100];
	sprintf(cmd,"%s input.graph",optimizer_name.c_str());
    int ret=system(cmd);

    if(ret==0)
    	ROS_INFO("Graph optimization complete!");
    else
    	ROS_WARN("Error: optimization failed");
}

/*
 * read output from optimizer and update the trajectory
 */
void GraphSlam::updateNodes()
{
      int id;
      double a, b, c;
      string tmpStr;
      string strTmp;

      ifstream ifs;
      string filename = "output_graph.txt";
      ifs.open(filename.c_str());
      if (!ifs.is_open()) {
    	  ROS_WARN("Can't open file output_graph.txt!");
          return;
        }

      while (getline(ifs, tmpStr))
      {
              stringstream ss;
              ss << tmpStr;

              ss >> strTmp;
              if (strTmp == vertex_name.c_str())
              {
                      ss >> id >> a >> b >> c;
                      geometry_msgs::Pose2D pose;
                      pose.x=a;
                      pose.y=b;
                      pose.theta=c;
                      nodes[id].pose = pose;
              }
      }
}

/*
 * cartesian difference over xy plane between two Transforms
 */
double GraphSlam::calcCartMismatch(tf::Transform a, tf::Transform b )
{
	return a.getOrigin().distance(b.getOrigin());
	//return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
}

/*
 * angular difference over z axis between two Transforms
 */
double GraphSlam::calcAngMismatch(tf::Transform a, tf::Transform b  )
{
	tf::Transform temp;
	temp=a.inverse()*b;
	double diff= fabs( getYawFromQuaternion(temp.getRotation()));

	return diff;
}

// reset the map by sending localized cloud with poseposition.z=-1 to the mapper
void GraphSlam::resetMap()
{
	occupancy_grid_utils::LocalizedCloud loc_cloud_reset;

	loc_cloud_reset.sensor_pose.position.z=-1.0;
	loc_cloud_reset.header.frame_id = fixed_frame_;
	loc_cloud_publisher_.publish(loc_cloud_reset);
	ROS_INFO("Re-initializing map");
}

/*
 * read nodes and publish localized point clouds for map construction
 */
void GraphSlam::updateMap()
{
	//bool sentNodes[nodes.size()];
	for(unsigned i=0; i< nodes.size(); i++)
		nodes[i].send=true;
		//sentNodes[i]=false;

	trajectory.poses.clear();
	trajectory_map.poses.clear();

	for(unsigned i = 0; i < nodes.size(); i++)
	{
		//bool send = true;

		for(unsigned j = 0; j < i; j++)
		{
			if( nodes[j].send == true )
			{
				tf::Transform tf_i, tf_j;
				createTfFromXYTheta(nodes[i].pose.x, nodes[i].pose.y, nodes[i].pose.theta, tf_i);
				createTfFromXYTheta(nodes[j].pose.x, nodes[j].pose.y, nodes[j].pose.theta, tf_j);

				if( calcCartMismatch(tf_i, tf_j) < 0.5 && calcAngMismatch(tf_i, tf_j) < 0.2 )
				//if( calcCartMismatch(tf_i, tf_j) < 0.1 && calcAngMismatch(tf_i, tf_j) < 0.1 )
				{
					ROS_DEBUG("Not sending node, distance: %f %f", calcCartMismatch(tf_i, tf_j),calcAngMismatch(tf_i, tf_j) );
					nodes[i].send = false;
					break;
				}

			}
		}

		// put the node into trajectory for visualization
		geometry_msgs::Pose pose;
		pose.position.x=nodes[i].pose.x;
		pose.position.y=nodes[i].pose.y;
		pose.position.z=0.0;
		tf::Quaternion	q=tf::createQuaternionFromYaw(nodes[i].pose.theta);
		pose.orientation.x=q.getX();
		pose.orientation.y=q.getY();
		pose.orientation.z=q.getZ();
		pose.orientation.w=q.getW();
		trajectory.poses.push_back(pose);

		// publish i-th localized point cloud (laser scan + pose) for mapping
		if(nodes[i].send)
		{
			sensor_msgs::PointCloud fixed_frame_cloud;
			laser_geometry::LaserProjection projector_;
			projector_.projectLaser(nodes[i].scanMsg, fixed_frame_cloud);
			occupancy_grid_utils::LocalizedCloud loc_cloud;
			loc_cloud.cloud.points = fixed_frame_cloud.points;
			tf::Transform robot_pose_tf;
			createTfFromXYTheta(pose.position.x, pose.position.y, getYawFromQuaternion(pose.orientation), robot_pose_tf);
			tf::Transform laser_pose_tf = robot_pose_tf * base_to_laser_;
			pose.position.x = laser_pose_tf.getOrigin().getX();
			pose.position.y = laser_pose_tf.getOrigin().getY();
			pose.position.z = laser_pose_tf.getOrigin().getZ();
			pose.orientation.x = laser_pose_tf.getRotation().getX();
			pose.orientation.y = laser_pose_tf.getRotation().getY();
			pose.orientation.z = laser_pose_tf.getRotation().getZ();
			pose.orientation.w = laser_pose_tf.getRotation().getW();
 			loc_cloud.sensor_pose = pose;
			loc_cloud.header.frame_id = fixed_frame_;
			// send node to mapper
			loc_cloud_publisher_.publish(loc_cloud);
			//sentNodes[i] = true;
			trajectory_map.poses.push_back(pose);
		}
	}
	// publish trajectory
	trajectory_publisher_.publish(trajectory);

	trajectory_map_publisher_.publish(trajectory_map);
}

/*
 * check trajectory consistency (return false if consecutive edges are too far apart
 * or there is a displacement over y coordinate)
 */
bool GraphSlam::checkNodes()
{
	for(unsigned i = 0; i < nodes.size()-1; i++)
	{
		double dx,dy, dx_star, dy_star, dist_star;

	    dx = nodes[i+1].pose.x - nodes[i].pose.x;
	    dy = nodes[i+1].pose.y - nodes[i].pose.y;
	    double theta = nodes[i].pose.theta;

		// relative displacement and distance between consecutive nodes
		// in the optimized trajectory
		dx_star = + cos(theta)*dx + sin(theta)*dy;
	    dy_star = - sin(theta)*dx + cos(theta)*dy;
		dist_star = sqrt(  dx_star*dx_star + dy_star*dy_star  );
		
		// we have a non-holonomic platform, therefore the y displacement
		// should be close to zero
		if(fabs(dy_star) > 0.3)
		{
			ROS_WARN("checkNodes(): Check trajectory consistency: loops rejected - y check");
			ROS_WARN("	dx_star %f dy_star %f dist_star %f",dx_star, dy_star, dist_star);

			return false;		
		}
		// robot speed is limited, then the maximum distance between 
		// consecutive nodes should be bounded
		if(dist_star > 1.5)
		{
			ROS_WARN("checkNodes(): Check trajectory consistency (this may depend on the frequency you use for initializing a node): loops rejected - dist check");
			ROS_WARN("	dx_star %f dy_star %f dist_star %f",dx_star, dy_star, dist_star);
			return false;		
		}
	}
	return true;
}

double GraphSlam::normalizeAngle(double angle)
{
	while(angle > M_PI)
		angle -= 2 * M_PI;
	while(angle < -M_PI)
		angle += 2 * M_PI;

	return angle;
}


double GraphSlam::calcVectorAvg(vector<double> vec)
{
	if(vec.size()==0) return 0;
	double sum=0;
	for (unsigned j=0; j < vec.size(); j++)
	{
		sum += vec[j];
	}
	double avg=sum/(double)vec.size();

	return avg;
}

/*
 * distance between two 2D poses
 */
geometry_msgs::Pose2D GraphSlam::calcPoseDiff2D(geometry_msgs::Pose2D a, geometry_msgs::Pose2D b)
{
	geometry_msgs::Pose2D result;
	double  delta_x, delta_y, delta_th;

    delta_x = b.x - a.x;
    delta_y = b.y - a.y;
    delta_th = normalizeAngle( b.theta-a.theta);

    double temp[2];
    temp[0] =    cos(a.theta)*delta_x + sin(a.theta)*delta_y;
    temp[1] =   - sin(a.theta)*delta_x + cos(a.theta)*delta_y;
    result.x = temp[0];
    result.y = temp[1];
    result.theta = delta_th;

    return result;
}

/*
 * check edges for consistency (return false if residuals are too high)
 */
bool GraphSlam::checkEdges()
{
	FILE *out=fopen("residual_errors.csv","a");
	vector<double> res_error_cart, res_error_ang;

	for(unsigned i=0; i< edges.size(); i++)
	{
		int id1=edges[i].id1;
		int id2=edges[i].id2;
		geometry_msgs::Pose2D deltaPStar=calcPoseDiff2D(nodes[id1].pose,nodes[id2].pose);

		tf::Transform deltaPStar_tf, deltaPMeas_tf;

		createTfFromXYTheta(deltaPStar.x, deltaPStar.y, deltaPStar.theta, deltaPStar_tf);
		deltaPMeas_tf = edges[i].t;
		res_error_cart.push_back(calcCartMismatch(deltaPStar_tf, deltaPMeas_tf));
		res_error_ang.push_back(calcAngMismatch(deltaPStar_tf, deltaPMeas_tf));
	}

	if(*max_element(res_error_cart.begin(), res_error_cart.end()) > 0.15 ||
			*max_element(res_error_ang.begin(), res_error_ang.end()) > 0.03 )
	{
		ROS_WARN("checkEdges(): loop closing discarded for residual error");
		return false;

	}

	ROS_DEBUG("checkEdges(): res_error_cart max: %f min: %f avg: %f",
			*max_element(res_error_cart.begin(), res_error_cart.end() ),
			*min_element(res_error_cart.begin(), res_error_cart.end()),
			calcVectorAvg(res_error_cart));
	ROS_DEBUG("checkEdges(): res_error_ang max: %f min: %f avg: %f",
			*max_element(res_error_ang.begin(), res_error_ang.end()),
			*min_element(res_error_ang.begin(), res_error_ang.end()),
			calcVectorAvg(res_error_ang));

	fprintf (out, "%f, %f, %f, %f, %f, %f\n",
			*max_element(res_error_cart.begin(), res_error_cart.end() ),
						*min_element(res_error_cart.begin(), res_error_cart.end()),
						calcVectorAvg(res_error_cart),
						*max_element(res_error_ang.begin(), res_error_ang.end()),
									*min_element(res_error_ang.begin(), res_error_ang.end()),
									calcVectorAvg(res_error_ang)
						);

	fclose(out);
	return true;
}

/*
 *  check if graph node b is visible from a
 */
bool GraphSlam::checkVisibility(Node a, Node b)
{
	geometry_msgs::Pose2D b_in_a = calcPoseDiff2D(a.pose, b.pose);
	double bearing = atan2( b_in_a.y, b_in_a.x);
	double distance = sqrt(b_in_a.x * b_in_a.x + b_in_a.y * b_in_a.y);

	int obstacleCounter = 0;
	if(bearing>a.scanMsg.angle_min && bearing < a.scanMsg.angle_max)
	{

		int rayIndex = (int)((bearing - a.scanMsg.angle_min) /a.scanMsg.angle_increment);

		int fov = (int) (0.05 / a.scanMsg.angle_increment);
		for (int i= rayIndex-fov; i<rayIndex+fov; i++ )
		{
			if(i < 0 || i > (int) (a.scanMsg.angle_max/a.scanMsg.angle_increment))
				continue;
			if( a.scanMsg.ranges[i] < distance )
				obstacleCounter++;
		}

		// if several rays confirm non-visibility loop is discarded
		if( obstacleCounter > (int) (0.05/a.scanMsg.angle_increment) )
		{
			ROS_DEBUG("checkVisibility(): bearing: %f min: %f max: %f, distance: %f, obstacleCounter: %d", bearing, a.scanMsg.angle_min,a.scanMsg.angle_max, distance, obstacleCounter);
			return false;
		}
	}
	else
	{
		if(b_in_a.theta < a.scanMsg.angle_min * 3/2  || b_in_a.theta > a.scanMsg.angle_max * 3/2)
		{
			ROS_DEBUG("checkVisibility(): loop discarded because poses are pointing in opposite directions");
			return false;
		}
	}

	return true;
}
