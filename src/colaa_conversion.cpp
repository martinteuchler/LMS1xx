#include "lms1xx/colaa_conversion.h"
#include <ros/ros.h>

void CoLaAConversion::fillMultiEchoLaserScan(sensor_msgs::MultiEchoLaserScan &scan, const ScanData &data)
{
  ROS_ASSERT(scan.ranges.size() ==  data.ch16bit.size());
  double start_angle = data.ch16bit[0].header.start_angle * M_PI / 180.0 / 10000.0 - M_PI / 2.0;
  double angle_increment = data.ch16bit[0].header.step_size * M_PI / 180.0 / 10000.0;

  scan.angle_increment = angle_increment;
  scan.angle_min = start_angle;
  scan.angle_max = start_angle + (data.ch16bit[0].data.size() - 1) * angle_increment;

  for (size_t i = 0; i < data.ch16bit.size(); ++i)
  {
    ROS_ASSERT(scan.ranges[i].echoes.size() ==  data.ch16bit[i].data.size());
    for (size_t k = 0; k < data.ch16bit[i].data.size(); ++k)
    {
      scan.ranges[i].echoes[k] = data.ch16bit[i].data[k] * 0.001 * data.ch16bit[i].header.scale_factor;
      scan.intensities[i].echoes[k] = data.ch8bit[i].data[k];
    }
  }
}

void CoLaAConversion::fillLaserScan(sensor_msgs::LaserScan &scan, const ScanData &data, size_t channel)
{
  ROS_ASSERT(data.ch16bit[channel].data.size() == scan.ranges.size());

  double start_angle = data.ch16bit[channel].header.start_angle * M_PI / 180.0 / 10000.0 - M_PI / 2.0;
  double angle_increment = data.ch16bit[channel].header.step_size * M_PI / 180.0 / 10000.0;

  scan.angle_increment = angle_increment;
  scan.angle_min = start_angle;
  scan.angle_max = start_angle + (data.ch16bit[0].data.size() - 1) * angle_increment;
  for (size_t k = 0; k < data.ch16bit[channel].data.size(); ++k)
  {
    scan.ranges[k] = data.ch16bit[channel].data[k] * 0.001 * data.ch16bit[channel].header.scale_factor;
    scan.intensities[k] = data.ch8bit[channel].data[k];
  }
}

void CoLaAConversion::fillPointCloud2(sensor_msgs::PointCloud2Iterator<float> &iter_x,
                                      sensor_msgs::PointCloud2Iterator<float> &iter_y,
                                      sensor_msgs::PointCloud2Iterator<float> &iter_z,
                                      sensor_msgs::PointCloud2Iterator<float> &iter_int,
                                      const ScanData &data, size_t echo)
{
  float layer_angle = CoLaALayers::getLayerAngle(
        static_cast<CoLaALayers::Layers>(data.header.status_info.layer_angle));
  float cosLA = cos(layer_angle);
  float sinLA = sin(layer_angle);
  double start_angle = data.ch16bit[echo].header.start_angle * M_PI / 180.0 / 10000.0 - M_PI / 2.0;
  double angle_increment = data.ch16bit[echo].header.step_size * M_PI / 180.0 / 10000.0;

  for (size_t i = 0; i < data.ch16bit[echo].data.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_int)
  {
    double dist = data.ch16bit[echo].data[i] * 0.001 * data.ch16bit[echo].header.scale_factor;
    double angle = start_angle + i * angle_increment;
    *iter_x = dist * cos(angle) * cosLA;
    *iter_y = dist * sin(angle) * cosLA;
    *iter_z = dist * sinLA;
    *iter_int = data.ch8bit[echo].data[i];
  }
}

void CoLaAConversion::fillPointCloud2MultiEcho(sensor_msgs::PointCloud2Iterator<float> &iter_x, sensor_msgs::PointCloud2Iterator<float> &iter_y, sensor_msgs::PointCloud2Iterator<float> &iter_z, sensor_msgs::PointCloud2Iterator<float> &iter_int, const ScanData &data)
{
  for (size_t i = 0; i < data.ch16bit.size(); ++i) {
    fillPointCloud2(iter_x, iter_y, iter_z, iter_int, data, i);
  }
}

template<>
size_t CoLaAConversion::findStrongestEcho<3>(const ScanData &data, size_t index)
{
  uint8_t d0 = data.ch8bit[0].data[index];
  uint8_t d1 = data.ch8bit[1].data[index];
  uint8_t d2 = data.ch8bit[2].data[index];
  if (d0 > d1 && d0 > d2)
    return 0;
  if (d1 > d2)
    return 1;
  return 2;
}
