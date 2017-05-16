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
#include <lms1xx/mrs1000.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#define DEG2RAD M_PI/180.0

// those are basically spherical coordinate
void dist_from_scan(float &out_x, float &out_y, float &out_z, float dist, float angle, float vertical_angle)
{
  out_x = dist * cos(angle) * cos(vertical_angle);
  out_y = dist * sin(angle) * cos(vertical_angle);
  out_z = dist * sin(vertical_angle);
}

int main(int argc, char **argv)
{
  // laser data
  MRS1000 laser;
  ScanConfig cfg;
  ScanOutputRange outputRange;
  ScanDataConfig dataCfg;
  sensor_msgs::PointCloud2 cloud;

  // parameters
  std::string host;
  std::string frame_id;
  int port;

  ros::init(argc, argv, "mrs1000");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<int>("port", port, 2111);

  cloud.header.frame_id = frame_id;
  cloud.header.stamp = ros::Time::now();
  cloud.height = 4; // 4 layers
  cloud.width = 275*4+1; // 275° with a resolution of 0.25° (+ 1)

  //Fill the fields using the PointCloudModifier
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32);
  //modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
  cloud.is_bigendian = false;
  cloud.is_dense = false;

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
    /* in the evaluation prototype, those functions are not (yet) available
    cfg = laser.get_scan_config();
    outputRange = laser.get_scan_output_range();

    if (cfg.scaningFrequency != 5000)
    {
      laser.disconnect();
      ROS_WARN("Unable to get laser output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }*/

    ROS_INFO("Connected to laser.");

    /* in the evaluation prototype, those functions are not (yet) available
    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser.set_scan_data_config(dataCfg);
    */
    /* in the evaluation prototype, those functions are not (yet) available
     * TODO: How does this work? Do I need to always start using SOPAS?
    ROS_DEBUG("Starting measurements.");
    laser.start_measurement();

    ROS_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    //while(1)
    //{
    CoLaA::Status stat = laser.query_status();
    ros::Duration(1.0).sleep();
    if (stat != CoLaA::Status::ReadyForMeasurement)
    {
      ROS_WARN("Laser not ready. Retrying initialization.");
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }
    /*if (stat == ready_for_measurement)
    {
      ROS_DEBUG("Ready status achieved.");
      break;
    }

      if (ros::Time::now() > ready_status_timeout)
      {
        ROS_WARN("Timed out waiting for ready status. Trying again.");
        laser.disconnect();
        continue;
      }

      if (!ros::ok())
      {
        laser.disconnect();
        return 1;
      }
    }*/

    ROS_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scanContinuous(true);

    double start_angle = -275.0/2.0*DEG2RAD;
    double angle_inc = 0.25*DEG2RAD;

    bool synced = false;
    int layers_received = 0;

    sensor_msgs::PointCloud2Iterator<float>iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>iter_int(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float>start_iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>start_iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>start_iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>start_iter_int(cloud, "intensity");

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      cloud.header.stamp = start;

      //scanDataLayerMRS data;
      scanDataLayerMRS data;
      ROS_DEBUG("Reading scan data.");

      if (laser.getScanData(&data))
      {

	// reset iterators and layer counter when receiving the first one, so we collect all layers in one cloud
        if (data.first)
        {
          iter_x = start_iter_x;
          iter_y = start_iter_y;
          iter_z = start_iter_z;
          iter_int = start_iter_int;
          synced = true;
          layers_received = 0;
        }


        if (!synced)
          continue;

	// if we would want to use all channels (i.e. all echos), we need to handle this better
	// i.e. without specifying this with fixed values...
	// for now, just use the first echo
	// for (size_t j = 0; j < (sizeof(data.channel)/sizeof(*(data.channel))); ++j)
        for (size_t j = 0; j < 1; ++j)
        {
          for (int i = 0; i < data.channel[j].data_len; ++i, ++iter_x, ++iter_y, ++iter_z, ++ iter_int)
          {
            dist_from_scan(*iter_x, *iter_y, *iter_z, data.channel[j].dist[i]*0.001, start_angle+i*angle_inc, -data.layer_angle*DEG2RAD);
            *iter_int = data.channel[j].rssi[i];
          }
        }
        ++layers_received;
        ROS_DEBUG("Publishing scan data.");
        if (layers_received == 4)
          cloud_pub.publish(cloud);
      }
      else
      {
        ROS_ERROR("Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();
    }

    laser.scanContinuous(false);
    /*
    laser.stopMeas();
    */
    laser.disconnect();
  }

  return 0;
}
