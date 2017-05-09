/*
 * lms_structs.h
 *
 *  Author: Konrad Banachowicz
 *          Mike Purvis <mpurvis@clearpathrobotics.com>
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

#ifndef LMS1XX_LMS_STRUCTS_H_
#define LMS1XX_LMS_STRUCTS_H_

#include <stdint.h>

/*!
* @class scanCfg
* @brief Structure containing scan configuration.
*
* @author Konrad Banachowicz
*/
struct scanCfg
{
  /*!
   * @brief Scanning frequency.
   * 1/100 Hz
   */
  int scaningFrequency;

  /*!
   * @brief number of active sensors
   * 1 for LMS1xx and LMS5xx, 1-4 for NAV310, LD-OEM15xx, LD-LRS36xx
   */
  int activeSensors;

  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanDataCfg
* @brief Structure containing scan data configuration.
*
* @author Konrad Banachowicz
*/
struct scanDataCfg
{

  /*!
   * @brief Output channels.
   * Defines which output channel is activated.
   */
  int outputChannel;

  /*!
   * @brief Remission.
   * Defines whether remission values are output.
   */
  bool remission;

  /*!
   * @brief Remission resolution.
   * Defines whether the remission values are output with 8-bit or 16bit resolution.
   * 8 bit: 0
   * 16 bit: 1
   */
  int resolution;

  /*!
   * @brief Encoders channels.
   * Defines which output channel is activated.
   */
  int encoder;

  /*!
   * @brief Position.
   * Defines whether position values are output.
   */
  bool position;

  /*!
   * @brief Device name.
   * Determines whether the device name is to be output.
   */
  bool deviceName;

  /*!
   * @brief Saved comment
   * Determines whether the saved comment is to be output.
   */
  bool comment;

  bool timestamp;

  /*!
   * @brief Output interval.
   * Defines which scan is output.
   *
   * 01 every scan\n
   * 02 every 2nd scan\n
   * ...\n
   * 50000 every 50000th scan
   */
  int outputInterval;
};

/*!
* @class outputRange
* @brief Structure containing scan output range configuration
*
* @author wpd
*/
struct scanOutputRange
{
  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanData
* @brief Structure containing single scan message.
*
* @author Konrad Banachowicz
*/
struct scanData
{

  /*!
   * @brief Number of samples in dist1.
   *
   */
  int dist_len1;

  /*!
   * @brief Radial distance for the first reflected pulse
   *
   */
  uint16_t dist1[1082];

  /*!
   * @brief Number of samples in dist2.
   *
   */
  int dist_len2;

  /*!
   * @brief Radial distance for the second reflected pulse
   *
   */
  uint16_t dist2[1082];

  /*!
   * @brief Number of samples in rssi1.
   *
   */
  int rssi_len1;

  /*!
   * @brief Remission values for the first reflected pulse
   *
   */
  uint16_t rssi1[1082];

  /*!
   * @brief Number of samples in rssi2.
   *
   */
  int rssi_len2;

  /*!
   * @brief Remission values for the second reflected pulse
   *
   */
  uint16_t rssi2[1082];
};

// TODO consider to use those for LMS as well
struct dataChannel
{
  int channel_nr;
  uint16_t data_len;
  uint16_t dist[1101];
  uint16_t rssi[1101];
};

struct scanDataLayerMRS
{
  bool first;
  int scan_nr;
  uint32_t time_since_startup;
  uint32_t transmission_duration;
  double layer_angle;
  int layer_nr; // starting from 0;
  dataChannel channel[3];
};

#endif  // LMS1XX_LMS_STRUCTS_H_
