#ifndef MRS1000_H
#define MRS1000_H
#include <LMS1xx/colaa.h>

class MRS1000 : public CoLaA
{
public:

protected:
  void parse_scan_data(char *buffer, void *scan_data);
};

#endif // MRS1000_H
