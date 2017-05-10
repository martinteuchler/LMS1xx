#ifndef COLAA_STRUCTS_H
#define COLAA_STRUCTS_H

#include "LMS1xx/parse_helpers.h"
#include <vector>

namespace CoLaAStatus
{
enum Status : uint8_t
{
  Undefined = 0,
  Initialisation = 1,
  Configuration = 2,
  Idle = 3,
  Rotated = 4,
  InPreparation = 5,
  Ready = 6,
  ReadyForMeasurement = 7,
  Error = 8 // Not from spec, failed to read status at all
};
}

namespace CoLaASopasError
{
enum SopasError
{
  Sopas_Ok = 0,
  Sopas_Error_METHODIN_ACCESDENIED = 1,
  Sopas_Error_METHODIN_UNKNOWNINDEX = 2,
  Sopas_Error_VARIABLE_UNKNONWINDEX = 3,
  Sopas_Error_LOCALCONDITIONFAILED = 4,
  Sopas_Error_INVALID_DATA = 5,
  Sopas_Error_UNKNOWN_Error = 6,
  Sopas_Error_BUFFER_OVERFLOW = 7,
  Sopas_Error_BUFFER_UNDERFLOW = 8,
  Sopas_Error_ERROR_UNKNOWN_TYPE = 9,
  Sopas_Error_VARIABLE_WRITE_aCCESSDENIED = 10,
  Sopas_Error_UNKNOWN_CMD_FOR_NAMESERVER = 11,
  Sopas_Error_UNKNOWN_COLA_COMMAND = 12,
  Sopas_Error_METHODIN_SERVER_BUSY = 13,
  Sopas_Error_FLEX_OUT_OF_BOUNDS = 14,
  Sopas_Error_EVENTREG_UNKNOWNINDEX = 15,
  Sopas_Error_COLA_A_VALUE_OVERFLOW = 16,
  Sopas_Error_COLA_A_INVALID_CHARACTER = 17,
  Sopas_Error_OSAI_NO_MESSAGE = 18,
  Sopas_Error_OSAI_NO_ANSWER_MESSAGE = 19,
  Sopas_Error_INTERNAL = 20,
  Sopas_Error_HubAddressCorrupted = 21,
  Sopas_Error_HubAddressDecoding = 22,
  Sopas_Error_HubAddressAddressExceeded = 23,
  Sopas_Error_HubAddressBlankExpected = 24,
  Sopas_Error_AsyncMethodsAreSuppressed = 25,
  Sopas_Error_ComplexArraysNotSupported = 26,
  PARSE_ERROR = 999 // Failed to parse error code
};


/**
 * @brief parse_error Parse error code from message
 * @param buf expected to be at the first character of the ASCII error code in the message.
 * @param twodigits Whether or not the error code has 2 digits. Can be determined from the message length.
 * If the message has two digits, it is assumed that buf has a size of at least 2.
 * @return The parsed error code or PARSE_ERROR if no error code could be parsed
 */
static SopasError parse_error(const char *buf, bool twodigits)
{
  if (!buf)
  {
    return SopasError::PARSE_ERROR;
  }
  int ones = buf[twodigits ? 1 : 0] - 48;
  int tens = 0;
  if (twodigits)
  {
    tens = buf[0] - 48;
  }
  if (ones < 0 || ones > 9 || tens < 0 || tens > 9)
  {
    return SopasError::PARSE_ERROR;
  }
  return static_cast<SopasError>(10 * tens + ones);
}
}

struct ScanConfig
{
  uint32_t scan_frequency;
  int16_t num_sectors;
  uint32_t angualr_resolution;
  int32_t start_ange;
  int32_t stop_angle;
};

struct ScanOutputRange
{
  int16_t num_sectors;
  uint32_t angular_resolution;
  int32_t start_angle;
  int32_t stop_angle;
};

/*!
  * @class scanDataCfg
  * @brief Structure containing scan data configuration.
  *
  * @author Konrad Banachowicz
  */
struct ScanDataConfig
{

  /*!
   * @brief Output channels.
   * Defines which output channel is activated.
   */
  int output_channel;

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
  bool device_name;

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
  int output_interval;
};

struct ScanDataHeader
{
  uint16_t version_number;

  // Device section
  struct
  {
    uint16_t device_number;
    uint32_t serial_number;
    uint8_t device_status_1;
    uint8_t device_status_2;
  } device;

  // Status info section
  struct
  {
    uint16_t telegram_counter;
    uint16_t scan_counter;
    uint32_t time_since_startup;
    uint32_t time_of_transmission;
    uint8_t status_digitalin_1; // LMS1xx, LMS5xx, TiM5xx only
    uint8_t status_digitalin_2; // LMS1xx, LMS5xx, TiM5xx only

    // 00 00 all outputs low
    // Rest is device dependant
    uint8_t status_digitalout_1;
    uint8_t status_digitalout_2;

    uint16_t reserved;
  } status_info;

  struct
  {
    uint32_t scan_frequency;
    uint32_t measurement_frequency;
  } frequencies;
};

struct ChannelDataHeader
{
  std::string contents;
  float scale_factor;
  float scale_factor_offset;
  int32_t start_angle; // for 16 bit channels this is uint32_t in the spec sheet, but has
  // negative values in the range regardless
  uint16_t step_size;
  uint16_t data_count;
};

template <typename T>
class ChannelData
{
public:
  ChannelDataHeader header;
  std::vector<T> data;

  static ChannelDataHeader parse_scan_data_channel_header(char **buf)
  {
    ChannelDataHeader header;
    next_token(buf, header.contents);
    next_token(buf, header.scale_factor);
    next_token(buf, header.scale_factor_offset);
    next_token(buf, header.start_angle);
    next_token(buf, header.step_size);
    next_token(buf, header.data_count);
    return header;
  }

  static std::vector<ChannelData<T> > parse_scan_data_channels(char **buf)
  {
    std::vector<ChannelData<T> > channels;
    uint16_t num_channels = 0;
    next_token(buf, num_channels);
    for (uint16_t channel = 0; channel < num_channels; ++channel)
    {
      ChannelData<T> chan;
      chan.header = parse_scan_data_channel_header(buf);
      chan.data.resize(chan.header.data_count);
      T data_n;
      for (uint16_t d = 0; d < chan.header.data_count; ++d)
      {
        next_token(buf, data_n);
        chan.data[d] = data_n;
      }
      channels.push_back(chan);
    }
    //TODO does this produce unnecessary copies of the data vectors?
    return channels;
  }
};

struct ScanData
{
  ScanDataHeader header;
  std::vector<ChannelData<uint16_t>> ch16bit;
  std::vector<ChannelData<uint8_t>> ch8bit;
};


#endif // COLAA_STRUCTS_H
