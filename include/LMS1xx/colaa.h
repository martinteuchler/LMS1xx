#ifndef COLAA_H
#define COLAA_H

#include <string>
#include <stdint.h>
#include "LMS1xx/lms_structs.h"

class LMSBuffer;

class CoLaA
{
public:
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

  CoLaA();
  ~CoLaA();

  /*!
  * @brief Connect to a device supporting the CoLaA protocol.
  * @param host device host name or ip address.
  * @param port device port number.
  */
  void connect(std::string host, int port);

  /*!
  * @brief Disconnect from CoLaA device.
  */
  void disconnect();

  /*!
  * @brief Get status of connection.
  * @returns connected or not.
  */
  bool is_connected() const;

  /*!
  * @brief Log into device
  * Increase privilege level, giving ability to change device configuration.
  */
  void login();

  /*!
  * @brief The device is returned to the measurement mode after configuration.
  *
  */
  void start_device();

  /*!
  * @brief Start measurements.
  * After receiving this command the unit starts spinning laser and measuring.
  */
  void start_measurement();

  /*!
  * @brief Stop measurements.
  * After receiving this command LMS1xx unit stop spinning laser and measuring.
  */
  void stop_measurement();

  /*!
  * @brief Set scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @param cfg structure containing scan configuration.
  */
  void set_scan_config(const scanCfg &cfg);

  /*!
  * @brief Set scan data configuration.
  * Set format of scan message returned by device.
  * @param cfg structure containing scan data configuration.
  */
  void set_scan_data_config(const scanDataCfg &cfg);

  /*!
  * @brief Get current scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanCfg structure.
  */
  scanCfg get_scan_config();

  /*!
  * @brief Save data permanently.
  * Parameters are saved in the EEPROM of the LMS and will also be available after the device is switched off and on again.
  *
  */
  void save_config();

  /*!
  * @brief Get current status of device.
  * This should be the same for all sensor types
  * @returns status of device.
  */
  Status query_status();

  /*!
  * @brief Get current output range configuration.
  * Get output range configuration :
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * Should be the same for all sensor types
  * @returns scanOutputRange structure.
  */
  scanOutputRange get_scan_output_range();

  /*!
  * @brief Start or stop continuous data acquisition.
  * After reception of this command device start or stop continuous data stream containing scan messages.
  * @param start true : start false : stop
  */
  void scan_continuous(bool start);

  /*!
  * @brief Receive single scan message.
  * @return true if scan was read successfully, false if error or timeout. False implies that higher level
  *         logic should take correct action such as reopening the connection.
  */
  // TODO: abstract scan_data
  bool get_scan_data(void *scan_data);

protected:
  std::string LOGIN_COMMAND;
  std::string LOGIN_USER_MAINT;
  std::string LOGIN_USER_AUTHORIZED;
  std::string LOGIN_USER_SERVICE;
  std::string LOGIN_PASS_MAINT;
  std::string LOGIN_PASS_AUTHORIZED;
  std::string LOGIN_PASS_SERVICE;

  std::string READ_SCAN_CFG_COMMAND;
  std::string SET_SCAN_CFG_COMMAND;
  std::string SET_SCAN_DATA_CFG_COMMAND;
  std::string SAVE_CONFIG_COMMAND;

  std::string START_MEASUREMENT_COMMAND;
  std::string STOP_MEASUREMENT_COMMAND;
  std::string START_CONT_COMMAND;

  std::string QUERY_STATUS_COMMAND;

  std::string READ_SCAN_OUTPUT_RANGE_COMMAND;

  std::string START_DEVICE_COMMAND;

  void do_login(std::string user_class, std::string password);

  virtual scanCfg parse_scan_cfg(const char *buf, size_t len);
  virtual std::string build_scan_cfg(const scanCfg &cfg) const;
  virtual std::string build_scan_data_cfg(const scanDataCfg &cfg) const;
  virtual void parse_scan_data(char *buffer, void *data) const;

private:
  void send_command(const std::string &command) const;
  void send_command(const char *command) const;
  bool read_back(char *buf, size_t &buflen);
  bool read_back();

  bool connected_;
  LMSBuffer *buffer_;
  int socket_fd_;
};

using LMS1xxx = CoLaA; // The base implementation is for LMS1xxx

#endif // COLAA_H
