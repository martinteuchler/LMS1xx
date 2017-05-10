#ifndef LMS5XX_H
#define LMS5XX_H

#include <stdint.h>
#include <string>
#include "LMS1xx/colaa.h"
#include "LMS1xx/colaa_structs.h"

class LMS5xx : public CoLaA
{
public:
  enum EchoFilter : uint8_t
  {
    FirstEcho = 0,
    AllEchos = 1,
    LastEcho = 2
  };

  LMS5xx();

  void set_echo_filter(EchoFilter filter);

protected:
  std::string build_scan_data_cfg_output_channel(int ch) const;
  std::string SET_ECHO_FILTER_COMMAND;
};

#endif // LMS5XX_H
