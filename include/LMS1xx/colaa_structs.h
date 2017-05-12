#ifndef COLAA_STRUCTS_H
#define COLAA_STRUCTS_H

#include "LMS1xx/parse_helpers.h"
#include <vector>

namespace CoLaAStatus
{
/**
 * @brief Sensor status as returned by CoLaA::query_status()
 */
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
/**
 * @brief Error codes returned by the sensor
 *
 * Used by CoLaA::read_back() and logged to console.
 */
enum SopasError
{
  Sopas_Ok = 0,
  Sopas_Error_METHODIN_ACCESDENIED = 1,
  Sopas_Error_METHODIN_UNKNOWNINDEX = 2,
  Sopas_Error_VARIABLE_UNKNONWINDEX = 3,
  Sopas_Error_LOCALCONDITIONFAILED = 4,
  Sopas_Error_INVALID_DATA = 5,
  Sopas_Error_UNKNOWN_ERROR = 6,
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
static SopasError parseError(const char *buf, bool twodigits)
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

/**
 * @brief Scan config
 */
struct ScanConfig
{
  /**
   * @brief Scan frequency in 1/100 of Hz
   */
  uint32_t scan_frequency;
  /**
   * @brief Number of configured sectors
   *
   * Should be 1 as support for multiple sectors is not implemented
   * in this driver
   */
  int16_t num_sectors;
  /**
   * @brief Angular resolution in 1/1000 deg
   */
  uint32_t angualar_resolution;
  /**
   * @brief Start angle in 1/1000 deg
   */
  int32_t start_angle;
  /**
   * @brief Stop angle in 1/1000 deg
   */
  int32_t stop_angle;
};

/**
 * @brief Scan output range
 */
struct ScanOutputRange
{
  /**
   * @brief Number of configured sectors
   *
   * Should be 1 as support for multiple sectors is not implemented
   * in this driver
   */
  int16_t num_sectors;
  /**
   * @brief Angular resolution in 1/1000 deg
   */
  uint32_t angular_resolution;
  /**
   * @brief Start angle in 1/1000 deg
   */
  int32_t start_angle;
  /**
   * @brief Stop angle in 1/1000 deg
   */
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

  /*!
   * @brief Saved comment
   * Determines whether the timestamp is to be output.
   */
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

/**
 * @brief Global header of the scan data
 */
struct ScanDataHeader
{
  /**
   * @brief Protocol version number, should be 1
   */
  uint16_t version_number;

  // Device section
  struct
  {
    /**
     * @brief Device number as defined with SOPAS
     */
    uint16_t device_number;
    /**
     * @brief Device serial number (factory set)
     */
    uint32_t serial_number;
    /**
     * @brief Device status bit 1
     * OK: 00 00
     * Error 00 01
     * Pollution warning: 00 02
     * Pollution error: 00 05
     */
    uint8_t device_status_1;
    /**
     * @brief Device status bit 2
     */
    uint8_t device_status_2;
  } device;

  // Status info section
  struct
  {
    /**
     * @brief Number of measurements finished in the scanner and given to the interface
     */
    uint16_t telegram_counter;
    /**
     * @brief Number of scans which were created in the device; counts how many scans were really done
     */
    uint16_t scan_counter;
    /**
     * @brief Time since power up starting with 0. In the output telegram this is the time at the zero index (-14 deg)
     * before the measurement itself starts.
     */
    uint32_t time_since_startup;
    /**
     * @brief Timestamp of scan completion in microseconds since boot
     */
    uint32_t time_of_transmission;
    /**
     * @brief Status of digital in, byte 1, low byte represents input 1
     * All inputs low: 00 00
     * All inputs high: 00 03
     */
    uint8_t status_digitalin_1; // LMS1xx, LMS5xx, TiM5xx only
    /**
     * @brief Status of digital in, byte 2
     */
    uint8_t status_digitalin_2; // LMS1xx, LMS5xx, TiM5xx only

    /**
     * @brief Status of digital outputs, byte 1
     * 00 00 all outputs low
     * Rest is device specific
     */
    uint8_t status_digitalout_1;
    /**
     * @brief Status of digital outputs, byte 2
     */
    uint8_t status_digitalout_2;

    /**
     * @brief Reserved
     */
    uint16_t reserved;
  } status_info;

  // Frequencies section
  struct
  {
    /**
     * @brief Scan frequency in 1/100 Hz
     */
    uint32_t scan_frequency;
    /**
     * @brief Measurement frequency in 1/100 Hz
     */
    uint32_t measurement_frequency;
  } frequencies;
};

/**
 * @brief Header for each 16 and 8 bit channel
 */
struct ChannelDataHeader
{
  /**
   * @brief Type of content, may be DIST1 - DIST5 or RSSI1 - RSSI5
   */
  std::string contents;
  /**
   * @brief Scale factor or factor of the measurement values (for the LMS5xx this depends on the angular resolution)
   */
  float scale_factor;
  /**
   * @brief Sets starting point of measurement, should be 0 for LMSxxx
   */
  float scale_factor_offset;
  /**
   * @brief Start angle in 1/1000 deg
   * This is defined as Uint_32 in the spec sheet but it can contain negative values
   */
  int32_t start_angle; // for 16 bit channels this is uint32_t in the spec sheet, but has
  /**
   * @brief Step size in 1/1000 deg
   */
  uint16_t step_size;
  /**
   * @brief Number of data points in this channel
   */
  uint16_t data_count;
};

/**
 * @brief Combines the header and data for one output channel
 * Template parameter should be uint16_t or uint8_t for 16 bit and 8 bit channels
 * respectively.
 */
template <typename T>
class ChannelData
{
public:
  ChannelDataHeader header;
  std::vector<T> data;

  /**
   * @brief Parse channel header from stream
   * @param buf the data stream
   * @return the parsed header
   */
  static ChannelDataHeader parseScanDataChannelHeader(char **buf)
  {
    ChannelDataHeader header;
    nextToken(buf, header.contents);
    nextToken(buf, header.scale_factor);
    nextToken(buf, header.scale_factor_offset);
    nextToken(buf, header.start_angle);
    nextToken(buf, header.step_size);
    nextToken(buf, header.data_count);
    return header;
  }

  /**
   * @brief Parse all channels of given type T
   * @param buf data stream
   * @return A vector containing all extracted data channels in this section
   */
  static std::vector<ChannelData<T> > parseScanDataChannels(char **buf)
  {
    std::vector<ChannelData<T> > channels;
    uint16_t num_channels = 0;
    nextToken(buf, num_channels);
    for (uint16_t channel = 0; channel < num_channels; ++channel)
    {
      ChannelData<T> chan;
      chan.header = parseScanDataChannelHeader(buf);
      chan.data.resize(chan.header.data_count);
      T data_n;
      for (uint16_t d = 0; d < chan.header.data_count; ++d)
      {
        nextToken(buf, data_n);
        chan.data[d] = data_n;
      }
      channels.push_back(chan);
    }
    //TODO does this produce unnecessary copies of the data vectors?
    return channels;
  }
};

/**
 * @brief Combines all scan data into one struct (excluding encoder data which we discard)
 */
struct ScanData
{
  /**
   * @brief Global header
   */
  ScanDataHeader header;
  /**
   * @brief 16 bit measurement channels
   */
  std::vector<ChannelData<uint16_t>> ch16bit;
  /**
   * @brief 8 bit measurement channels
   */
  std::vector<ChannelData<uint8_t>> ch8bit;
};


#endif // COLAA_STRUCTS_H
