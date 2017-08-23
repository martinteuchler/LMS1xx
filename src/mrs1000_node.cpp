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

#include <lms1xx/mrs1000.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  MRS1000 laser;
  ScanConfig cfg;
  ScanOutputRange output_range;
  ScanDataConfig data_cfg;
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

    //laser.stopMeasurement();

    ROS_DEBUG("Logging in to laser.");
    laser.login();

    cfg = laser.getScanConfig();
    output_range = laser.getScanOutputRange();

    ROS_DEBUG("Laser configuration: scaningFrequency %d, activeSensors %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scan_frequency, cfg.num_sectors, cfg.angualar_resolution, cfg.start_angle, cfg.stop_angle);
    ROS_DEBUG("Laser output range: angleResolution %d, startAngle %d, stopAngle %d",
              output_range.angular_resolution, output_range.start_angle, output_range.stop_angle);

    ROS_INFO("Connected to laser.");

    data_cfg.output_channel = 7; // 1 + 2 + 3
    data_cfg.remission = true;
    data_cfg.resolution = 0;
    data_cfg.encoder = 0;
    data_cfg.position = false;
    data_cfg.device_name = false;
    data_cfg.comment = false;
    data_cfg.timestamp = 1;
    data_cfg.output_interval = 1; // all scans

    ROS_DEBUG("Setting scan data configuration.");
    laser.setScanDataConfig(data_cfg);

    ROS_DEBUG("Setting echo configuration");
    laser.setEchoFilter(CoLaAEchoFilter::AllEchoes);

    ROS_DEBUG("Setting application mode");
    laser.enableRangingApplication();

    laser.saveConfig();

    ROS_INFO("Starting device...");
    laser.startDevice(); // Log out to properly re-enable system after config

    laser.startMeasurement();
    //ros::Duration(20.0).sleep();

    ROS_INFO("...started. Starting continuous measurements");
    laser.scanContinuous(true);
    //laser.requestLastScan();

    sensor_msgs::PointCloud2Iterator<float>iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>iter_int(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float>start_iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>start_iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>start_iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>start_iter_int(cloud, "intensity");
    bool synced = false;

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      cloud.header.stamp = start;

      //scanDataLayerMRS data;
      ScanData data;
      ROS_DEBUG("Reading scan data.");

      if (laser.getScanData(&data))
      {
        // reset iterators when receiving the first one, so we collect all layers in one cloud
        if (data.header.status_info.layer_angle == CoLaALayers::Layer2)
        {
          iter_x = start_iter_x;
          iter_y = start_iter_y;
          iter_z = start_iter_z;
          iter_int = start_iter_int;
          synced = true;
        }


        if (!synced)
          continue;

        float layerAngle = CoLaALayers::getLayerAngle(
              static_cast<CoLaALayers::Layers>(data.header.status_info.layer_angle));
        float cosLA = cos(layerAngle);
        float sinLA = sin(layerAngle);
        double startAngle = data.ch16bit[0].header.start_angle * M_PI / 180.0 / 10000.0;
        double angleIncrement = data.ch16bit[0].header.step_size * M_PI / 180.0 / 10000.0;

        for (size_t i = 0; i < data.ch16bit[0].data.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_int)
        {
          double dist = data.ch16bit[0].data[i] * 0.001 * data.ch16bit[0].header.scale_factor;
          double angle = startAngle + i * angleIncrement;
          *iter_x = dist * cos(angle) * cosLA;
          *iter_y = dist * sin(angle) * cosLA;
          *iter_z = dist * sinLA;
          *iter_int = data.ch8bit[0].data[i];
        }

        // Check if this is the last layer of the msg
        if (data.header.status_info.layer_angle == CoLaALayers::Layer4)
        {
          ROS_DEBUG("Publishing scan data.");
          cloud_pub.publish(cloud);
        }
      }
      else
      {
        ROS_ERROR("Laser timed out on delivering scan, attempting to reinitialize.");
        ros::Duration(10.0).sleep();
        break;
      }

      ros::spinOnce();
    }

    laser.disconnect();
  }

  return 0;
}
