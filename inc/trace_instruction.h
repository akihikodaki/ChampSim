/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRACE_INSTRUCTION_H
#define TRACE_INSTRUCTION_H

#include <limits>

// special registers that help us identify branches
namespace champsim
{
constexpr char REG_STACK_POINTER = 6;
constexpr char REG_FLAGS = 25;
constexpr char REG_INSTRUCTION_POINTER = 26;
} // namespace champsim

#define TRACE_TYPE_INVALID 0
#define TRACE_TYPE_X86 1
#define TRACE_TYPE_RISCV 2
#define TRACE_TYPE_CLOUDSUITE 3

// instruction format
constexpr std::size_t NUM_INSTR_DESTINATIONS_SPARC = 4;
constexpr std::size_t NUM_INSTR_DESTINATIONS = 2;
constexpr std::size_t NUM_INSTR_SOURCES = 4;

struct input_instr {
  // instruction pointer or PC (Program Counter)
  unsigned long long ip;

  // branch info
  unsigned char is_branch;
  unsigned char branch_taken;

  unsigned char destination_registers[NUM_INSTR_DESTINATIONS]; // output registers
  unsigned char source_registers[NUM_INSTR_SOURCES];           // input registers

  unsigned long long destination_memory[NUM_INSTR_DESTINATIONS]; // output memory
  unsigned long long source_memory[NUM_INSTR_SOURCES];           // input memory
};

struct cloudsuite_instr {
  // instruction pointer or PC (Program Counter)
  unsigned long long ip;

  // branch info
  unsigned char is_branch;
  unsigned char branch_taken;

  unsigned char destination_registers[NUM_INSTR_DESTINATIONS_SPARC]; // output registers
  unsigned char source_registers[NUM_INSTR_SOURCES];                 // input registers

  unsigned long long destination_memory[NUM_INSTR_DESTINATIONS_SPARC]; // output memory
  unsigned long long source_memory[NUM_INSTR_SOURCES];                 // input memory

  unsigned char asid[2];
};

#define NUM_INSTR_DESTINATIONS_RISCV 1
#define NUM_INSTR_SOURCES_RISCV 3

struct riscv_instr {
  unsigned long long int ip;                                               // instruction pointer (program counter) value
  unsigned long long int destination_memory[NUM_INSTR_DESTINATIONS_RISCV]; // output memory
  unsigned long long int source_memory[NUM_INSTR_SOURCES_RISCV];           // input memory
  unsigned long long ret_val;
  unsigned int inst;
  // unsigned short op;
  unsigned char is_branch;    // is this branch
  unsigned char branch_taken; // if so, is this taken

  unsigned char destination_registers[NUM_INSTR_DESTINATIONS_RISCV]; // output registers
  unsigned char source_registers[NUM_INSTR_SOURCES_RISCV];           // input registers
};

#endif
