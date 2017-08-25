/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <csignal>
#include <cstdio>
#include <lms1xx/lms5xx.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include "lms1xx/colaa_conversion.h"

constexpr double DEG2RAD = M_PI/180.0;
constexpr size_t ALL_ECHOES_COUNT = 5;

void usage()
{
  std::cout << "LMS5xx_node" << std::endl;
  std::cout << "LMS5xx series laser scanner node." << std::endl;
  std::cout << "If configured for all echoes, a MultiEchoLaserScan message will be published"
            " on the \"multi_echo\" topic in addition to the regular, single echo \"scan\" topic." << std::endl << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "    host      The IP of the scanner" << std::endl;
  std::cout << "    port      The port to connect on" << std::endl;
  std::cout << "    frame_id  Frame id of the laser, defaults to \"laser\"." << std::endl;
  std::cout << "    echoes    One of \"first\", \"last\" or \"all\"." << std::endl;
  std::cout << "    range     Maximum sensor range in m (default 80)" << std::endl;
}

bool setup(LMS5xx &laser, sensor_msgs::LaserScan &scan_msg, sensor_msgs::MultiEchoLaserScan &multi_scan_msg,
           const CoLaAEchoFilter::EchoFilter echo_mode, double max_range)
{
  ScanConfig cfg;
  ScanOutputRange output_range;
  ScanDataConfig dataCfg;

  ROS_DEBUG("Logging in to laser.");
  laser.login();
  cfg = laser.getScanConfig();
  output_range = laser.getScanOutputRange();

  if (cfg.scan_frequency != 5000)
  {
    laser.disconnect();
    ROS_WARN("Unable to get laser output range. Retrying.");
    ros::Duration(1).sleep();
    return false;
  }

  ROS_INFO("Connected to laser.");

  ROS_DEBUG("Laser configuration: scaningFrequency %d, activeSensors %d, angleResolution %d, startAngle %d, stopAngle %d",
           cfg.scan_frequency, cfg.num_sectors, cfg.angualar_resolution, cfg.start_angle, cfg.stop_angle);
  ROS_DEBUG("Laser output range: angleResolution %d, startAngle %d, stopAngle %d",
           output_range.angular_resolution, output_range.start_angle, output_range.stop_angle);

  scan_msg.range_min = 0.01;
  scan_msg.range_max = max_range;
  scan_msg.scan_time = 100.0 / cfg.scan_frequency;

  multi_scan_msg.range_min = 0.01;
  multi_scan_msg.range_max = max_range;
  multi_scan_msg.scan_time = 100.0 / cfg.scan_frequency;

  ROS_DEBUG_STREAM("Device resolution is " << (double)output_range.angular_resolution / 10000.0 << " degrees.");
  ROS_DEBUG_STREAM("Device frequency is " << (double)cfg.scan_frequency / 100.0 << " Hz");

  int angle_range = output_range.stop_angle - output_range.start_angle;
  int num_values = angle_range / output_range.angular_resolution ;
  if (angle_range % output_range.angular_resolution == 0)
  {
    // Include endpoint
    ++num_values;
  }
  scan_msg.ranges.resize(num_values);
  scan_msg.intensities.resize(num_values);

  multi_scan_msg.ranges.resize(ALL_ECHOES_COUNT);
  multi_scan_msg.intensities.resize(ALL_ECHOES_COUNT);
  assert(multi_scan_msg.ranges.size() == multi_scan_msg.intensities.size());
  for (size_t i = 0; i < multi_scan_msg.ranges.size(); ++i)
  {
    multi_scan_msg.ranges[i].echoes.resize(num_values);
    multi_scan_msg.intensities[i].echoes.resize(num_values);
  }

  scan_msg.time_increment =
      (output_range.angular_resolution / 10000.0)
      / 360.0
      / (cfg.scan_frequency / 100.0);

  multi_scan_msg.time_increment =
      (output_range.angular_resolution / 10000.0)
      / 360.0
      / (cfg.scan_frequency / 100.0);

  ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

  dataCfg.output_channel = 5; // Is ignored for LMS5xx
  dataCfg.remission = true;
  dataCfg.resolution = 0;
  dataCfg.encoder = 0;
  dataCfg.position = false;
  dataCfg.device_name = false;
  dataCfg.comment = false;
  dataCfg.timestamp = false;
  dataCfg.output_interval = 1;

  ROS_DEBUG("Setting echo filter.");
  laser.setEchoFilter(echo_mode);

  ROS_DEBUG("Setting scan data configuration.");
  laser.setScanDataConfig(dataCfg);

  ROS_DEBUG("Saving configuration.");
  laser.saveConfig();
  return true;
}

int main(int argc, char **argv)
{
  if (argc == 2)
  {
    std::string arg(argv[1]);
    if (arg == "-h" || arg == "--help")
    {
      usage();
      return 0;
    }
  }

  // laser data
  LMS5xx laser;
  sensor_msgs::LaserScan scan_msg;
  sensor_msgs::MultiEchoLaserScan multi_scan_msg;

  // parameters
  std::string host;
  std::string frame_id;
  int port;
  std::string echoes;
  CoLaAEchoFilter::EchoFilter echo_mode = CoLaAEchoFilter::AllEchoes;
  double max_range = 80;

  ros::init(argc, argv, "lms5xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher multi_pub;

  n.param<std::string>("host", host, "192.168.0.1");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<int>("port", port, 2111);
  n.param<std::string>("echoes", echoes, "all");
  n.param<double>("range", max_range, 80);

  if (echoes == std::string("first"))
  {
    echo_mode = CoLaAEchoFilter::FirstEcho;
  }
  else if (echoes == std::string("last"))
  {
    echo_mode = CoLaAEchoFilter::LastEcho;
  }
  else if (echoes == std::string("all"))
  {
    multi_pub = nh.advertise<sensor_msgs::MultiEchoLaserScan>("multi_echo", 1);
    echo_mode = CoLaAEchoFilter::AllEchoes;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid echoes parameter " << echoes << "\nValid parameters: all, first, last");
    return 1;
  }

  if (host.empty() || port < 0 || port > 65535)
  {
    ROS_ERROR_STREAM("Invalid connection configuration: host \"" << host << "\" port \"" << port << "\"!");
    return 1;
  }

  if (max_range <= 0)
  {
    ROS_ERROR_STREAM("Range must be positive!");
    return 1;
  }

  scan_msg.header.frame_id = frame_id;
  multi_scan_msg.header.frame_id = frame_id;

  while (ros::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    if (!setup(laser, scan_msg, multi_scan_msg, echo_mode, max_range))
    {
      continue;
    }

    ROS_DEBUG("Starting measurements.");
    laser.startMeasurement();

    ROS_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    ros::Duration(1.0).sleep();
    CoLaAStatus::Status stat = laser.queryStatus();
    if (stat != CoLaAStatus::ReadyForMeasurement)
    {
      ROS_WARN("Laser not ready (Current state: %d). Retrying initialization.", stat);
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scanContinuous(true);

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      multi_scan_msg.header.stamp = start;
      ++multi_scan_msg.header.seq;

      ScanData data;
      ROS_DEBUG("Reading scan data.");
      if (laser.getScanData(&data))
      {
        // Configured echo or first echo if "all" is selected
        CoLaAConversion::fillLaserScan(scan_msg, data);
        ROS_DEBUG("Publishing single scan data.");
        scan_pub.publish(scan_msg);

        // The multi-echo message if all echoes are selected
        if (echo_mode == CoLaAEchoFilter::AllEchoes)
        {

          CoLaAConversion::fillMultiEchoLaserScan(multi_scan_msg, data);
          ROS_DEBUG("Publishing multi scan data.");
          multi_pub.publish(multi_scan_msg);
        }
      }
      else
      {
        ROS_ERROR("Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();
    }

    laser.scanContinuous(false);
    laser.stopMeasurement();
    laser.disconnect();
  }

  return 0;
}
