#ifndef LMS5XX_H
#define LMS5XX_H

#include <stdint.h>
#include <string>
#include <LMS1xx/colaa.h>
#include <LMS1xx/colaa_structs.h>

/**
 * @brief Specialises CoLaA base implementation for the LMS5xx series of scanners
 */
class LMS5xx : public CoLaA
{
public:
  /**
   * @brief Echo return configuration
   */
  enum EchoFilter : uint8_t
  {
    FirstEcho = 0,
    AllEchoes = 1,
    LastEcho = 2
  };

  LMS5xx();

  /**
   * @brief Configures the echo return of the sensor
   * @param filter Which echoes to return
   */
  void setEchoFilter(EchoFilter filter);

protected:
  /**
   * @brief Channel data is handled by set_echo_filter for this sensor
   * @param ch
   * @return
   */
  std::string buildScanDataCfgOutputChannel(int ch) const;
  std::string SET_ECHO_FILTER_COMMAND;
};

#endif // LMS5XX_H
