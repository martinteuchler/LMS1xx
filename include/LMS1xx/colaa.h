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

  void login();

  void start_device();

  void start_measurement();

  void stop_measurement();

  void set_scan_config(const scanCfg &cfg);

  void set_scan_data_config(const scanDataCfg &cfg);

  scanCfg get_scan_config();

  void save_config();

  // This should be the same for all sensors
  Status query_status();

  // Should be same for all
  scanOutputRange get_scan_output_range();

  void scan_continuous(bool start);

  bool get_scan_data(scanData *scan_data);

  void printTest();

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
  virtual void parse_scan_data(char *buffer, scanData *data) const;

private:
  void send_command(const std::string &command) const;
  void send_command(const char *command) const;
  bool read_back(char *buf, size_t &buflen);
  bool read_back();

  bool connected_;
  LMSBuffer *buffer_;
  int socket_fd_;
};

#endif // COLAA_H
