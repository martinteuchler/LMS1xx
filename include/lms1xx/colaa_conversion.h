#ifndef COLAA_CONVERSION_H
#define COLAA_CONVERSION_H

#include <lms1xx/colaa_structs.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace CoLaAConversion
{
void fillMultiEchoLaserScan(sensor_msgs::MultiEchoLaserScan &scan, const ScanData &data);
void fillLaserScan(sensor_msgs::LaserScan &scan, const ScanData &data, size_t channel = 0);
void fillPointCloud2(sensor_msgs::PointCloud2Iterator<float> &iter_x,
                     sensor_msgs::PointCloud2Iterator<float> &iter_y,
                     sensor_msgs::PointCloud2Iterator<float> &iter_z,
                     sensor_msgs::PointCloud2Iterator<float> &iter_int,
                     const ScanData &data, size_t echo = 0);
void fillPointCloud2MultiEcho(sensor_msgs::PointCloud2Iterator<float> &iter_x,
                     sensor_msgs::PointCloud2Iterator<float> &iter_y,
                     sensor_msgs::PointCloud2Iterator<float> &iter_z,
                     sensor_msgs::PointCloud2Iterator<float> &iter_int,
                     const ScanData &data);

template <size_t echo_count>
size_t findStrongestEcho(const ScanData &data, size_t index)
{
  size_t max_echo = 0;
  uint8_t max_val = data.ch8bit[0].data[index];
  for (size_t i = 1; i < echo_count; ++i)
  {
    if (data.ch8bit[i].data[index] > max_val)
    {
      max_val = data.ch8bit[i].data[index];
      max_echo = i;
    }
  }
  return max_echo;
}

template <>
size_t findStrongestEcho<3>(const ScanData &data, size_t index);

template <size_t echo_count>
void fillPointCloud2Strongest(sensor_msgs::PointCloud2Iterator<float> &iter_x,
                     sensor_msgs::PointCloud2Iterator<float> &iter_y,
                     sensor_msgs::PointCloud2Iterator<float> &iter_z,
                     sensor_msgs::PointCloud2Iterator<float> &iter_int,
                     const ScanData &data)
{
  float layer_angle = CoLaALayers::getLayerAngle(
        static_cast<CoLaALayers::Layers>(data.header.status_info.layer_angle));
  float cosLA = cos(layer_angle);
  float sinLA = sin(layer_angle);
  double start_angle = data.ch16bit[0].header.start_angle * M_PI / 180.0 / 10000.0 - M_PI / 2.0;
  double angle_increment = data.ch16bit[0].header.step_size * M_PI / 180.0 / 10000.0;

  for (size_t i = 0; i < data.ch16bit[0].data.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_int)
  {
    size_t echo = findStrongestEcho<echo_count>(data, i);
    double dist = data.ch16bit[echo].data[i] * 0.001 * data.ch16bit[echo].header.scale_factor;
    double angle = start_angle + i * angle_increment;
    *iter_x = dist * cos(angle) * cosLA;
    *iter_y = dist * sin(angle) * cosLA;
    *iter_z = dist * sinLA;
    *iter_int = data.ch8bit[echo].data[i];
  }
}

}

#endif // COLAA_CONVERSION_H
