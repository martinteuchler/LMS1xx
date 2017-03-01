

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "LMS1xx/MRS1000.h"
#include "console_bridge/console.h"

// Get scan data for one layer
bool MRS1000::getScanData(scanDataLayerMRS *scan_data)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    logDebug("entering select()", tv.tv_usec);
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    logDebug("returned %d from select()", retval);
    if (retval)
    {
      buffer_.readFrom(socket_fd_);

      // Will return pointer if a complete message exists in the buffer,
      // otherwise will return null.
      char* buffer_data = buffer_.getNextBuffer();

      if (buffer_data)
      {
        parseScanLayer(buffer_data, scan_data);
        buffer_.popLastBuffer();
        return true;
      }
    }
    else
    {
      // Select timed out or there was an fd error.
      return false;
    }
  }
}

bool MRS1000::parseScanLayer(char *buffer, scanDataLayerMRS *scan_data)
{
  char* tok = strtok(buffer, " "); //Type of command
  tok = strtok(NULL, " "); //Command
  tok = strtok(NULL, " "); //VersionNumber
  tok = strtok(NULL, " "); //DeviceNumber
  tok = strtok(NULL, " "); //Serial number
  tok = strtok(NULL, " "); //DeviceStatus TODO
  tok = strtok(NULL, " "); //MessageCounter TODO
  tok = strtok(NULL, " "); //ScanCounter TODO
  sscanf(tok, "%u", &scan_data->scan_nr);
  tok = strtok(NULL, " "); //Time since startup
  sscanf(tok, "%u", &scan_data->time_since_startup);
  tok = strtok(NULL, " "); //TransmissionDuration
  sscanf(tok, "%u", &scan_data->transmission_duration);
  tok = strtok(NULL, " "); //InputStatus
  tok = strtok(NULL, " "); //OutputStatus
  tok = strtok(NULL, " "); //Layer angle
  int layer_angle_int;
  sscanf(tok, "%d", &layer_angle_int);
  scan_data->layer_angle = (double) layer_angle_int / 100.0;

  if (scan_data->layer_angle == 2.5)
  {
    scan_data->layer_nr = 0;
  } else if (scan_data->layer_angle == 0)
  {
    scan_data->layer_nr = 1;
  } else if (scan_data->layer_angle == -2.5)
  {
    scan_data->layer_nr = 2;
  } else if (scan_data->layer_angle == -5)
  {
    scan_data->layer_nr = 3;
  } else
  {
    logError("layer angle not supported: %f shold be one of (2.5, 0, -2.5, -5)", scan_data->layer_angle);
    return false;
  }

  tok = strtok(NULL, " "); //ScanningFrequency
  // TODO
  tok = strtok(NULL, " "); //MeasurementFrequency
  // TODO

  // why are those here?? Not part of protocol!
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");

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
  if (NumberChannels16Bit != 3)
  {
    logError("Parsed %d 16 bit channels, but RMS expects 3!", NumberChannels16Bit);
    return false;
  }

  dataChannel* p_channel = 0;
  for (int i = 0; i < NumberChannels16Bit; i++)
  {
    int type = -1; // 0 DIST1 1 DIST2 2 DIST3

    char content[6];
    tok = strtok(NULL, " "); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "DIST3"))
    {
      type = 2;
    }
    else
    {
      logError("Parsed  data type %s of 16 bit channel %u not supported", content, i);
      return false;
    }

    // save pointer to channel
    p_channel = &scan_data->channel[type];
    p_channel->channel_nr = type;

    tok = strtok(NULL, " "); //ScalingFactor
    double scale_factor = 1.0; // TODO
    // TODO read scale factor
    // TODO compare with config
    tok = strtok(NULL, " "); //ScalingOffset
    tok = strtok(NULL, " "); //Starting angle
    tok = strtok(NULL, " "); //Angular step width
    tok = strtok(NULL, " "); //NumberData

    sscanf(tok, "%u", &p_channel->data_len);
    logDebug("NumberData : %d", p_channel->data_len);

    // READ data from 16 bit channel
    for (int i = 0; i < p_channel->data_len; i++)
    {
      int dat;
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);
      p_channel->dist[i] = dat * scale_factor;
    }
  }

  tok = strtok(NULL, " "); //NumberChannels8Bit
  int NumberChannels8Bit;
  sscanf(tok, "%d", &NumberChannels8Bit);
  if (NumberChannels8Bit != p_channel->data_len)
  {
    logError("Number of 8 bit channels (%u) not 3!", NumberChannels8Bit);
    return false;
  }

  logDebug("NumberChannels8Bit : %d\n", NumberChannels8Bit);

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
      return false;
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
      return false;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat; // todo adapt size?
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);

      p_channel->rssi[i] = dat; // TODO add scaling factor!
    }
  }
  return true;
}
