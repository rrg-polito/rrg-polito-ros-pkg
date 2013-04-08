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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GraphSlam");
//  log4cxx::LoggerPtr my_logger =
//             log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//  my_logger->setLevel(
//             ros::console::g_level_lookup[ros::console::levels::Info]);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GraphSlam laser_scan_matcher(nh, nh_private);
  ros::spin();
  return 0;
}
