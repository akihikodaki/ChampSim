#ifndef REGFILE_H
#define REGFILE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include "champsim_constants.h"

class REGFILE
{
protected:
  std::string init_fname;

public:
  uint64_t data[64] = {0};

  void set_init_fname(const std::string fname);
  void init();

  uint64_t read(uint8_t addr);
  void write(uint8_t addr, uint64_t wdata);
  // constructor
  REGFILE() {}
};

#endif
