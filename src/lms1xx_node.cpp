/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <lms1xx/colaa.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  ScanConfig cfg;
  ScanOutputRange output_range;
  ScanDataConfig dataCfg;
  sensor_msgs::LaserScan scan_msg;

  // parameters
  std::string host;
  std::string frame_id;
  int port;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<int>("port", port, 2111);

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

    ROS_DEBUG("Logging in to laser.");
    laser.login();
    cfg = laser.getScanConfig();
    output_range = laser.getScanOutputRange();

    if (cfg.scan_frequency != 5000)
    {
      laser.disconnect();
      ROS_WARN("Unable to get laser output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_INFO("Connected to laser.");

    ROS_DEBUG("Laser configuration: scaningFrequency %d, activeSensors %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scan_frequency, cfg.num_sectors, cfg.angualar_resolution, cfg.start_angle, cfg.stop_angle);
    ROS_DEBUG("Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
              output_range.angular_resolution, output_range.start_angle, output_range.stop_angle);

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scan_frequency;
    scan_msg.angle_increment = (double)output_range.angular_resolution / 10000.0 * DEG2RAD;
    scan_msg.angle_min = (double)output_range.start_angle / 10000.0 * DEG2RAD - M_PI / 2;
    scan_msg.angle_max = (double)output_range.stop_angle / 10000.0 * DEG2RAD - M_PI / 2;

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

    scan_msg.time_increment =
      (output_range.angular_resolution / 10000.0)
      / 360.0
      / (cfg.scan_frequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    dataCfg.output_channel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.device_name = false;
    dataCfg.comment = false;
    dataCfg.timestamp = false;
    dataCfg.output_interval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser.setScanDataConfig(dataCfg);

    ROS_DEBUG("Starting measurements.");
    laser.startMeasurement();

    ROS_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    //while(1)
    //{
    CoLaAStatus::Status stat = laser.queryStatus();
    ros::Duration(1.0).sleep();
    if (stat != CoLaAStatus::ReadyForMeasurement)
    {
      ROS_WARN("Laser not ready (Current state: %d). Retrying initialization.", stat);
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }

    ROS_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scanContinuous(true);

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      ScanData data;
      ROS_DEBUG("Reading scan data.");
      if (laser.getScanData(&data))
      {
        for (size_t k = 0; k < data.ch16bit[0].data.size(); ++k)
        {
          scan_msg.ranges[k] = data.ch16bit[0].data[k] * 0.001;
          scan_msg.intensities[k] = data.ch16bit[1].data[k];
        }
        ROS_DEBUG("Publishing scan data.");
        scan_pub.publish(scan_msg);
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
