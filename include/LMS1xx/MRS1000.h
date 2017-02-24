#ifndef MRS1000_H
#define MRS1000_H
#include <LMS1xx/LMS1xx.h>

class MRS1000 : public LMS1xx
{
public:
  bool getScanData(scanDataLayerMRS* scan_data);

protected:
  static bool parseScanData(char* buf, scanDataLayerMRS* scan_data);
};

#endif // MRS1000_H
