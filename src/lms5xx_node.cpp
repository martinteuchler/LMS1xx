#include <csignal>
#include <cstdio>
#include <LMS1xx/lms5xx.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

#define DEG2RAD M_PI/180.0

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
}

bool setup(LMS5xx &laser, sensor_msgs::LaserScan &scan_msg, sensor_msgs::MultiEchoLaserScan &multi_scan_msg,
           const LMS5xx::EchoFilter echo_mode)
{
  ScanConfig cfg;
  ScanOutputRange output_range;
  ScanDataConfig dataCfg;

  ROS_DEBUG("Logging in to laser.");
  laser.login();
  cfg = laser.get_scan_config();
  output_range = laser.get_scan_output_range();

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
  scan_msg.range_max = 20.0; // TODO: value?
  scan_msg.scan_time = 100.0 / cfg.scan_frequency;
  scan_msg.angle_increment = (double)output_range.angular_resolution / 10000.0 * DEG2RAD;
  scan_msg.angle_min = (double)output_range.start_angle / 10000.0 * DEG2RAD - M_PI / 2;
  scan_msg.angle_max = (double)output_range.stop_angle / 10000.0 * DEG2RAD - M_PI / 2;

  multi_scan_msg.range_min = 0.01;
  multi_scan_msg.range_max = 20.0; // TODO: value?
  multi_scan_msg.scan_time = 100.0 / cfg.scan_frequency;
  multi_scan_msg.angle_increment = (double)output_range.angular_resolution / 10000.0 * DEG2RAD;
  multi_scan_msg.angle_min = (double)output_range.start_angle / 10000.0 * DEG2RAD - M_PI / 2;
  multi_scan_msg.angle_max = (double)output_range.stop_angle / 10000.0 * DEG2RAD - M_PI / 2;

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

  multi_scan_msg.ranges.resize(5);
  multi_scan_msg.intensities.resize(5);
  for (size_t i = 0; i < 5; ++i)
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

  dataCfg.output_channel = 5;
  dataCfg.remission = true;
  dataCfg.resolution = 0;
  dataCfg.encoder = 0;
  dataCfg.position = false;
  dataCfg.device_name = false;
  dataCfg.comment = false;
  dataCfg.timestamp = false;
  dataCfg.output_interval = 1;

  ROS_DEBUG("Setting echo filter.");
  laser.set_echo_filter(echo_mode);

  ROS_DEBUG("Setting scan data configuration.");
  laser.set_scan_data_config(dataCfg);

  ROS_DEBUG("Saving configuration.");
  laser.save_config();
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
  LMS5xx::EchoFilter echo_mode = LMS5xx::EchoFilter::AllEchoes;

  ros::init(argc, argv, "lms5xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::Publisher multi_pub = nh.advertise<sensor_msgs::MultiEchoLaserScan>("multi_echo", 1);

  n.param<std::string>("host", host, "192.168.0.1");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<int>("port", port, 2111);
  n.param<std::string>("echoes", echoes, "all");

  if (echoes == std::string("first"))
  {
    echo_mode = LMS5xx::EchoFilter::FirstEcho;
  }
  else if (echoes == std::string("last"))
  {
    echo_mode = LMS5xx::EchoFilter::LastEcho;
  }
  else if (echoes != std::string("all"))
  {
    ROS_ERROR_STREAM("Invalid echoes parameter " << echoes << "\nValid parameters: all, first, last");
    return 1;
  }

  scan_msg.header.frame_id = frame_id;
  multi_scan_msg.header.frame_id = frame_id;

  while (ros::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.is_connected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    if (!setup(laser, scan_msg, multi_scan_msg, echo_mode))
    {
      continue;
    }

    ROS_DEBUG("Starting measurements.");
    laser.start_measurement();

    ROS_DEBUG("Starting device.");
    laser.start_device(); // Log out to properly re-enable system after config

    ros::Duration(1.0).sleep();
    CoLaAStatus::Status stat = laser.query_status();
    if (stat != CoLaAStatus::ReadyForMeasurement)
    {
      ROS_WARN("Laser not ready (Current state: %d). Retrying initialization.", stat);
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scan_continuous(true);

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      multi_scan_msg.header.stamp = start;
      ++multi_scan_msg.header.seq;

      ScanData data;
      ROS_DEBUG("Reading scan data.");
      if (laser.get_scan_data(&data))
      {
        // Configured echo or first echo if "all" is selected
        for (size_t k = 0; k < data.ch16bit[0].data.size(); ++k)
        {
          scan_msg.ranges[k] = data.ch16bit[0].data[k] * 0.001;
          scan_msg.intensities[k] = data.ch8bit[0].data[k];
        }
        ROS_DEBUG("Publishing single scan data.");
        scan_pub.publish(scan_msg);

        // The multi-echo message if all echoes are selected
        if (echo_mode == LMS5xx::EchoFilter::AllEchoes)
        {
          for (size_t i = 0; i < data.ch16bit.size(); ++i)
          {
            for (size_t k = 0; k < data.ch16bit[i].data.size(); ++k)
            {
              multi_scan_msg.ranges[i].echoes[k] = data.ch16bit[i].data[k] * 0.001;
              multi_scan_msg.intensities[i].echoes[k] = data.ch8bit[i].data[k];
            }
          }
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

    laser.scan_continuous(false);
    laser.stop_measurement();
    laser.disconnect();
  }

  return 0;
}
