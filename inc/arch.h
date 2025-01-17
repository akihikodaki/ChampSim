#ifndef ARCH_H
#define ARCH_H

#include <cstdint>

namespace champsim
{
struct Arch {
  uint8_t reg_stack_pointer;
  uint8_t reg_flags;
  uint8_t reg_instruction_pointer;
};

extern Arch arch;
} // namespace champsim

#endif
