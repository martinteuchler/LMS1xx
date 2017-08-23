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
                     const ScanData &data);
}

#endif // COLAA_CONVERSION_H
