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

#include "LMS1xx/MRS1000.h"
#include <cstring>
#include <console_bridge/console.h>

void MRS1000::parseScanData(char *buffer, void *data)
{
  scanDataLayerMRS *scan_data = (scanDataLayerMRS *)data;
  char* tok = strtok(buffer, " "); //Type of command
  tok = strtok(NULL, " "); //Command
  tok = strtok(NULL, " "); //VersionNumber
  tok = strtok(NULL, " "); //DeviceNumber
  tok = strtok(NULL, " "); //Serial number
  tok = strtok(NULL, " "); //DeviceStatus1 TODO
  tok = strtok(NULL, " "); //DeviceStatus2 TODO
  tok = strtok(NULL, " "); //MessageCounter TODO
  tok = strtok(NULL, " "); //ScanCounter TODO
  sscanf(tok, "%u", &scan_data->scan_nr);
  tok = strtok(NULL, " "); //Time since startup
  sscanf(tok, "%u", &scan_data->time_since_startup);
  tok = strtok(NULL, " "); //TransmissionDuration
  sscanf(tok, "%u", &scan_data->transmission_duration);
  tok = strtok(NULL, " "); //InputStatus1
  tok = strtok(NULL, " "); //InputStatus2
  tok = strtok(NULL, " "); //OutputStatus1
  tok = strtok(NULL, " "); //OutputStatus2
  tok = strtok(NULL, " "); //Layer angle

  uint16_t layer_angle_hex;
  sscanf(tok, "%hx", &layer_angle_hex);
  int16_t layer_angle_int = static_cast<int16_t>(layer_angle_hex);
  scan_data->layer_angle = static_cast<double>(layer_angle_int) / 100.0;

  if (scan_data->layer_angle == 2.5)
  {
    scan_data->layer_nr = 0;
    scan_data->first = false;
  } else if (scan_data->layer_angle == 0)
  {
    scan_data->layer_nr = 1;
    scan_data->first = true;
  } else if (scan_data->layer_angle == -2.5)
  {
    scan_data->layer_nr = 2;
    scan_data->first = false;
  } else if (scan_data->layer_angle == -5)
  {
    scan_data->layer_nr = 3;
    scan_data->first = false;
  } else
  {
    logError("layer angle not supported: %f shold be one of (2.5, 0, -2.5, -5)", scan_data->layer_angle);
    return;
  }
  tok = strtok(NULL, " "); //ScanningFrequency
  // TODO
  tok = strtok(NULL, " "); //MeasurementFrequency
  // TODO

  tok = strtok(NULL, " "); //NumberEncoders
  int NumberEncoders;
  sscanf(tok, "%d", &NumberEncoders);
  for (int i = 0; i < NumberEncoders; i++)
  {
    tok = strtok(NULL, " "); //EncoderPosition
    tok = strtok(NULL, " "); //EncoderSpeed
  }

  tok = strtok(NULL, " "); //NumberChannels16Bit
  int NumberChannels16Bit;
  sscanf(tok, "%d", &NumberChannels16Bit);

  dataChannel* p_channel = 0;
  for (int i = 0; i < NumberChannels16Bit; i++)
  {
    int type = -1; // 0 DIST1 1 DIST2 2 DIST3
    int channel = 0;

    char content[6];
    tok = strtok(NULL, " "); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = 0;
      channel = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
      channel = 1;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = 2;
      channel = 0;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 3;
      channel = 1;
    }
    else
    {
      logError("Parsed  data type %s of 16 bit channel %u not supported", content, i);
      return;
    }

    // save pointer to channel
    p_channel = &scan_data->channel[channel];
    p_channel->channel_nr = channel;

    tok = strtok(NULL, " "); //ScalingFactor
    double scale_factor = 1.0; // TODO
    // TODO read scale factor
    // TODO compare with config
    tok = strtok(NULL, " "); //ScalingOffset
    tok = strtok(NULL, " "); //Starting angle
    tok = strtok(NULL, " "); //Angular step width
    tok = strtok(NULL, " "); //NumberData

    sscanf(tok, "%hx", &p_channel->data_len);

    // READ data from 16 bit channel
    for (int i = 0; i < p_channel->data_len; i++)
    {
      int dat;
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);
      if (type == 0 || type == 1)
        p_channel->dist[i] = dat * scale_factor;
      else if (type == 2 || type == 3)
        p_channel->rssi[i] = dat * scale_factor;
    }
  }

  tok = strtok(NULL, " "); //NumberChannels8Bit
  int NumberChannels8Bit;
  sscanf(tok, "%d", &NumberChannels8Bit);

  logDebug("NumberChannels8Bit : %d\n", NumberChannels8Bit);

  // this is zero anyways, so ignore for now...
  for (int i = 0; i < NumberChannels8Bit; i++)
  {
    int type = -1;
    dataChannel* p_channel = 0;
    char content[6];
    tok = strtok(NULL, " "); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "RSSI1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "RSSI3"))
    {
      type = 2;
    }
    else
    {
      logError("Parsed  data type %s of 8 bit channel %u not supported", content, i);
      return;
    }

    // save pointer to channel
    p_channel = &scan_data->channel[type];
    p_channel->channel_nr = type;

    // TODO read! and check if consistent with above
    tok = strtok(NULL, " "); //ScalingFactor
    tok = strtok(NULL, " "); //ScalingOffset
    tok = strtok(NULL, " "); //Starting angle
    tok = strtok(NULL, " "); //Angular step width
    tok = strtok(NULL, " "); //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);
    logDebug("NumberData : %d\n", NumberData);

    if (NumberData != p_channel->data_len)
    {
      logError("Number of dist (%u) and rssi (%u) values do not match!", p_channel->data_len, NumberData);
      return;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat; // todo adapt size?
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);

      p_channel->rssi[i] = dat; // TODO add scaling factor!
    }
  }
}
