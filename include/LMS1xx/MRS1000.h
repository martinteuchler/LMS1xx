#ifndef MRS1000_H
#define MRS1000_H
#include <LMS1xx/colaa.h>

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


class MRS1000 : public CoLaA
{
public:

protected:
  void parseScanData(char *buffer, void *scan_data);
};

#endif // MRS1000_H
