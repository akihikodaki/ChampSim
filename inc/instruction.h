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

#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "branch_type.h"
#include "prefetch.h"
#include "regfile.h"
#include "trace_instruction.h"
#include <capstone/capstone.h>
#include <unordered_map>

#define PRINT_TRACE_IP 0xffffffffffffffff
#define nPRINT_TRACE_INFO
#define nCOLLECT_INFO
#define nCOLLECT_PATTERN
#define nDECODE_COLLECT_INFO
#define nMISS_COLLECT_INFO
#define nIC_LENGTH_INFO
#define nCOLLECT_SAME_SRC
#define nCOLLECT_USELESS

#define LOAD_TYPE_NONE 0
#define LOAD_TYPE_ORIGINAL_STRIDE 1
#define LOAD_TYPE_ORIGINAL_PT_CHAIN 2
#define LOAD_TYPE_INFECTED_STRIDE 3
#define LOAD_TYPE_INFECTED_PT_CHAIN 4

extern uint64_t total_inst_num;
extern uint64_t total_load_num;
extern uint8_t regfile_load_type[64];
extern uint8_t regfile_op[64];

extern bool start_print;

typedef struct load_info {
  uint64_t total_count;
  uint64_t last_vaddr;

  int64_t stride;
  uint8_t stride_conf;
  uint64_t stride_count;

  uint8_t pt_conf;
  int64_t pt_offset;
  uint64_t last_res;
  uint64_t pt_count;

  uint64_t dma_count;
} load_info_t;

typedef struct decode_load_info {
  uint64_t total_count;

  uint64_t stride_count;
  uint64_t ima_single_count;
  uint64_t ima_double_count;
  uint64_t ima_complex_count;
} decode_load_info_t;

extern std::unordered_map<uint64_t, load_info_t> pc_info;

extern REGFILE regfile[NUM_CPUS];

struct ooo_model_instr {
  uint64_t instr_id = 0;
  uint64_t ip = 0;
  uint64_t event_cycle = 0;

  bool is_branch = 0;
  bool branch_taken = 0;
  bool branch_prediction = 0;
  bool branch_mispredicted = 0; // A branch can be mispredicted even if the direction prediction is correct when the predicted target is not correct

  std::array<uint8_t, 2> asid = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};

  uint8_t branch_type = NOT_BRANCH;
  uint64_t branch_target = 0;

  uint8_t dib_checked = 0;
  uint8_t fetched = 0;
  uint8_t decoded = 0;
  uint8_t scheduled = 0;
  uint8_t executed = 0;

  unsigned completed_mem_ops = 0;
  int num_reg_dependent = 0;

  std::vector<uint8_t> destination_registers = {}; // output registers
  std::vector<uint8_t> source_registers = {};      // input registers

  std::vector<uint64_t> destination_memory = {};
  std::vector<uint64_t> source_memory = {};

  // For IDM
  uint32_t inst = 0;
  IDM_OP op = IDM_INVALID;
  uint64_t source_reg_val[NUM_INSTR_SOURCES] = {};
  uint64_t ret_val = 0; // if destination_registers isn't 0, the register will be updated to ret_val.
  uint8_t ls_size = 0;
#ifdef MISS_COLLECT_INFO
  uint8_t load_type = IDM_INVALID;
#endif

  // these are indices of instructions in the ROB that depend on me
  std::vector<std::reference_wrapper<ooo_model_instr>> registers_instrs_depend_on_me;

  void prinf_info()
  {

    char msg[32];
    printf("op: %d, pc: %#lx, inst: %s, 0x%08x\n", op, this->ip, msg, this->inst);

    // register
    printf("source_registers     : %02d", this->source_registers[0]);
    for (uint64_t i = 1; i < NUM_INSTR_SOURCES; i++) {
      printf(", %02d", this->source_registers[i]);
    }
    printf("\ndestination_registers: %02d", this->destination_registers[0]);
    for (uint64_t i = 1; i < NUM_INSTR_DESTINATIONS_SPARC; i++) {
      printf(", %02d", this->destination_registers[i]);
    }

    // memory
    printf("\nsource_memory     : %#lx", this->source_memory[0]);
    for (uint64_t i = 1; i < NUM_INSTR_SOURCES; i++) {
      printf(", %#lx", this->source_memory[i]);
    }
    printf("\ndestination_memory: %#lx", this->destination_memory[0]);
    for (uint64_t i = 1; i < NUM_INSTR_DESTINATIONS_SPARC; i++) {
      printf(", %#lx", this->destination_memory[i]);
    }

    // source register value
    printf("\nsource_reg_val: %#lx", this->source_reg_val[0]);
    for (uint64_t i = 1; i < NUM_INSTR_SOURCES; i++) {
      printf(", %#lx", this->source_reg_val[i]);
    }

    // printf("imm[0]: %d, imm[1]: %d\n", this->imm[0], this->imm[1]);
    printf("\nret_val: 0x%016lx\n", this->ret_val);
    printf("is_branch: %d, branch_taken: %d\n\n", this->is_branch, this->branch_taken);
  }

private:
  template <typename T>
  ooo_model_instr(T instr, std::array<uint8_t, 2> local_asid) : ip(instr.ip), is_branch(instr.is_branch), branch_taken(instr.branch_taken), asid(local_asid)
  {
    std::remove_copy(std::begin(instr.destination_registers), std::end(instr.destination_registers), std::back_inserter(this->destination_registers), 0);
    std::remove_copy(std::begin(instr.source_registers), std::end(instr.source_registers), std::back_inserter(this->source_registers), 0);
    std::remove_copy(std::begin(instr.destination_memory), std::end(instr.destination_memory), std::back_inserter(this->destination_memory), 0);
    std::remove_copy(std::begin(instr.source_memory), std::end(instr.source_memory), std::back_inserter(this->source_memory), 0);

    bool writes_sp = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_STACK_POINTER);
    bool writes_ip = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_INSTRUCTION_POINTER);
    bool reads_sp = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_STACK_POINTER);
    bool reads_flags = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_FLAGS);
    bool reads_ip = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_INSTRUCTION_POINTER);
    bool reads_other = std::count_if(std::begin(source_registers), std::end(source_registers), [](uint8_t r) {
      return r != champsim::REG_STACK_POINTER && r != champsim::REG_FLAGS && r != champsim::REG_INSTRUCTION_POINTER;
    });

    // determine what kind of branch this is, if any
    if (!reads_sp && !reads_flags && writes_ip && !reads_other) {
      // direct jump
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_DIRECT_JUMP;
    } else if (!reads_sp && !reads_flags && writes_ip && reads_other) {
      // indirect branch
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_INDIRECT;
    } else if (!reads_sp && reads_ip && !writes_sp && writes_ip && reads_flags && !reads_other) {
      // conditional branch
      is_branch = true;
      branch_taken = instr.branch_taken; // don't change this
      branch_type = BRANCH_CONDITIONAL;
    } else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && !reads_other) {
      // direct call
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_DIRECT_CALL;
    } else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && reads_other) {
      // indirect call
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_INDIRECT_CALL;
    } else if (reads_sp && !reads_ip && writes_sp && writes_ip) {
      // return
      is_branch = true;
      branch_taken = true;
      branch_type = BRANCH_RETURN;
    } else if (writes_ip) {
      // some other branch type that doesn't fit the above categories
      is_branch = true;
      branch_taken = instr.branch_taken; // don't change this
      branch_type = BRANCH_OTHER;
    } else {
      branch_taken = false;
    }
  }

public:
  ooo_model_instr(uint8_t cpu, input_instr instr) : ooo_model_instr(instr, {cpu, cpu}) {}
  ooo_model_instr(uint8_t, cloudsuite_instr instr) : ooo_model_instr(instr, {instr.asid[0], instr.asid[1]}) {}

  ooo_model_instr(uint8_t cpu, riscv_instr instr) : ooo_model_instr(instr, {cpu, cpu})
  {
    this->ret_val = instr.ret_val;
    this->inst = instr.inst;

    branch_type = instr.is_branch;

    csh handle;
    cs_insn* insn;
    auto code = reinterpret_cast<const uint8_t*>(&instr.inst);
    auto size = sizeof(instr.inst);

    auto err = cs_open(CS_ARCH_RISCV, CS_MODE_RISCV64, &handle);
    if (err) {
      throw std::runtime_error("cs_open: "s + cs_strerror(err));
    }

    err = cs_option(handle, CS_OPT_DETAIL, CS_OPT_ON);
    if (err) {
      cs_close(&handle);
      throw std::runtime_error("cs_option: "s + cs_strerror(err));
    }

    auto success = cs_disasm(handle, code, size, instr.ip, 1, &insn);
    cs_close(&handle);

    if (!success) {
      throw std::invalid_argument("cs_disasm: "s + cs_strerror(cs_errno(handle)));
    }

    switch (insn->id) {
    case RISCV_INS_ADDIW:
    case RISCV_INS_ADDW:
    case RISCV_INS_C_ADDIW:
    case RISCV_INS_C_ADDW:
      op = IDM_ADD_W;
      break;

    case RISCV_INS_ADD:
    case RISCV_INS_ADDI:
    case RISCV_INS_C_ADD:
    case RISCV_INS_C_ADDI:
      op = IDM_ADD_D;
      break;

    case RISCV_INS_SUBW:
    case RISCV_INS_C_SUBW:
      op = IDM_SUB_W;
      break;

    case RISCV_INS_SUB:
    case RISCV_INS_C_SUB:
      op = IDM_SUB_D;
      break;

    case RISCV_INS_AND:
    case RISCV_INS_ANDI:
    case RISCV_INS_C_AND:
    case RISCV_INS_C_ANDI:
      op = IDM_AND;
      break;

    case RISCV_INS_C_OR:
    case RISCV_INS_OR:
    case RISCV_INS_ORI:
      op = IDM_OR;
      break;

    case RISCV_INS_C_XOR:
    case RISCV_INS_XORI:
      op = IDM_XOR;
      break;

    case RISCV_INS_MULW:
      op = IDM_MUL_W;
      break;

    case RISCV_INS_MUL:
      op = IDM_MUL_D;
      break;

    case RISCV_INS_SRLIW:
    case RISCV_INS_SRLW:
      op = IDM_SRL_W;
      break;

    case RISCV_INS_C_SRLI:
    case RISCV_INS_SRL:
    case RISCV_INS_SRLI:
      op = IDM_SRL_D;
      break;

    case RISCV_INS_SLLW:
    case RISCV_INS_SLLIW:
      op = IDM_SLLI_W;
      break;

    case RISCV_INS_C_SLLI:
    case RISCV_INS_SLL:
    case RISCV_INS_SLLI:
      op = IDM_SLLI_D;
      break;

    case RISCV_INS_SRAIW:
    case RISCV_INS_SRAW:
      op = IDM_SRA_W;
      break;

    case RISCV_INS_C_SRAI:
    case RISCV_INS_SRA:
    case RISCV_INS_SRAI:
      op = IDM_SRA_D;
      break;

    case RISCV_INS_LB:
      op = IDM_LD_B;
      break;

    case RISCV_INS_LBU:
      op = IDM_LD_BU;
      break;

    case RISCV_INS_LH:
      op = IDM_LD_H;
      break;

    case RISCV_INS_LHU:
      op = IDM_LD_HU;
      break;

    case RISCV_INS_C_FLW:
    case RISCV_INS_C_FLWSP:
    case RISCV_INS_LR_W:
    case RISCV_INS_LR_W_AQ:
    case RISCV_INS_LR_W_AQ_RL:
    case RISCV_INS_LR_W_RL:
    case RISCV_INS_LW:
      op = IDM_LD_W;
      break;

    case RISCV_INS_LWU:
      op = IDM_LD_WU;
      break;

    case RISCV_INS_C_FLD:
    case RISCV_INS_C_FLDSP:
    case RISCV_INS_FLD:
    case RISCV_INS_LD:
    case RISCV_INS_LR_D:
    case RISCV_INS_LR_D_AQ:
    case RISCV_INS_LR_D_AQ_RL:
    case RISCV_INS_LR_D_RL:
      op = IDM_LD_D;
      break;
    }

    switch (insn->id) {
    case RISCV_INS_LB:
    case RISCV_INS_LBU:
    case RISCV_INS_SB:
      ls_size = 1;

    case RISCV_INS_LH:
    case RISCV_INS_LHU:
    case RISCV_INS_SH:
      ls_size = 2;
      break;

    case RISCV_INS_AMOADD_W:
    case RISCV_INS_AMOADD_W_AQ:
    case RISCV_INS_AMOADD_W_AQ_RL:
    case RISCV_INS_AMOADD_W_RL:
    case RISCV_INS_AMOAND_W:
    case RISCV_INS_AMOAND_W_AQ:
    case RISCV_INS_AMOAND_W_AQ_RL:
    case RISCV_INS_AMOAND_W_RL:
    case RISCV_INS_AMOMAXU_W:
    case RISCV_INS_AMOMAXU_W_AQ:
    case RISCV_INS_AMOMAXU_W_AQ_RL:
    case RISCV_INS_AMOMAXU_W_RL:
    case RISCV_INS_AMOMAX_W:
    case RISCV_INS_AMOMAX_W_AQ:
    case RISCV_INS_AMOMAX_W_AQ_RL:
    case RISCV_INS_AMOMAX_W_RL:
    case RISCV_INS_AMOMINU_W:
    case RISCV_INS_AMOMINU_W_AQ:
    case RISCV_INS_AMOMINU_W_AQ_RL:
    case RISCV_INS_AMOMINU_W_RL:
    case RISCV_INS_AMOMIN_W:
    case RISCV_INS_AMOMIN_W_AQ:
    case RISCV_INS_AMOMIN_W_AQ_RL:
    case RISCV_INS_AMOMIN_W_RL:
    case RISCV_INS_AMOOR_W:
    case RISCV_INS_AMOOR_W_AQ:
    case RISCV_INS_AMOOR_W_AQ_RL:
    case RISCV_INS_AMOOR_W_RL:
    case RISCV_INS_AMOSWAP_W:
    case RISCV_INS_AMOSWAP_W_AQ:
    case RISCV_INS_AMOSWAP_W_AQ_RL:
    case RISCV_INS_AMOSWAP_W_RL:
    case RISCV_INS_AMOXOR_W:
    case RISCV_INS_AMOXOR_W_AQ:
    case RISCV_INS_AMOXOR_W_AQ_RL:
    case RISCV_INS_AMOXOR_W_RL:
    case RISCV_INS_C_FLW:
    case RISCV_INS_C_FLWSP:
    case RISCV_INS_C_FSW:
    case RISCV_INS_C_FSWSP:
    case RISCV_INS_C_LW:
    case RISCV_INS_C_LWSP:
    case RISCV_INS_C_SW:
    case RISCV_INS_C_SWSP:
    case RISCV_INS_FLW:
    case RISCV_INS_FSW:
    case RISCV_INS_LR_W:
    case RISCV_INS_LR_W_AQ:
    case RISCV_INS_LR_W_AQ_RL:
    case RISCV_INS_LR_W_RL:
    case RISCV_INS_LW:
    case RISCV_INS_LWU:
    case RISCV_INS_SC_W:
    case RISCV_INS_SC_W_AQ:
    case RISCV_INS_SC_W_AQ_RL:
    case RISCV_INS_SC_W_RL:
    case RISCV_INS_SW:
      ls_size = 4;
      break;

    case RISCV_INS_AMOADD_D:
    case RISCV_INS_AMOADD_D_AQ:
    case RISCV_INS_AMOADD_D_AQ_RL:
    case RISCV_INS_AMOADD_D_RL:
    case RISCV_INS_AMOAND_D:
    case RISCV_INS_AMOAND_D_AQ:
    case RISCV_INS_AMOAND_D_AQ_RL:
    case RISCV_INS_AMOAND_D_RL:
    case RISCV_INS_AMOMAXU_D:
    case RISCV_INS_AMOMAXU_D_AQ:
    case RISCV_INS_AMOMAXU_D_AQ_RL:
    case RISCV_INS_AMOMAXU_D_RL:
    case RISCV_INS_AMOMAX_D:
    case RISCV_INS_AMOMAX_D_AQ:
    case RISCV_INS_AMOMAX_D_AQ_RL:
    case RISCV_INS_AMOMAX_D_RL:
    case RISCV_INS_AMOMINU_D:
    case RISCV_INS_AMOMINU_D_AQ:
    case RISCV_INS_AMOMINU_D_AQ_RL:
    case RISCV_INS_AMOMINU_D_RL:
    case RISCV_INS_AMOMIN_D:
    case RISCV_INS_AMOMIN_D_AQ:
    case RISCV_INS_AMOMIN_D_AQ_RL:
    case RISCV_INS_AMOMIN_D_RL:
    case RISCV_INS_AMOOR_D:
    case RISCV_INS_AMOOR_D_AQ:
    case RISCV_INS_AMOOR_D_AQ_RL:
    case RISCV_INS_AMOOR_D_RL:
    case RISCV_INS_AMOSWAP_D:
    case RISCV_INS_AMOSWAP_D_AQ:
    case RISCV_INS_AMOSWAP_D_AQ_RL:
    case RISCV_INS_AMOSWAP_D_RL:
    case RISCV_INS_AMOXOR_D:
    case RISCV_INS_AMOXOR_D_AQ:
    case RISCV_INS_AMOXOR_D_AQ_RL:
    case RISCV_INS_AMOXOR_D_RL:
    case RISCV_INS_C_FLD:
    case RISCV_INS_C_FLDSP:
    case RISCV_INS_C_FSD:
    case RISCV_INS_C_FSDSP:
    case RISCV_INS_C_LD:
    case RISCV_INS_C_LDSP:
    case RISCV_INS_C_SD:
    case RISCV_INS_C_SDSP:
    case RISCV_INS_FLD:
    case RISCV_INS_FSD:
    case RISCV_INS_LD:
    case RISCV_INS_LR_D:
    case RISCV_INS_LR_D_AQ:
    case RISCV_INS_LR_D_AQ_RL:
    case RISCV_INS_LR_D_RL:
    case RISCV_INS_SC_D:
    case RISCV_INS_SC_D_AQ:
    case RISCV_INS_SC_D_AQ_RL:
    case RISCV_INS_SC_D_RL:
    case RISCV_INS_SD:
      ls_size = 8;
      break;
    }

    auto source_reg_valp = source_reg_val;

    for (uint8_t i = 0; i < insn->detail->regs_read_count; i++) {
      *source_reg_valp = regfile[cpu].read(insn->detail->riscv.operands[i].reg);
      source_reg_valp++;
    }

    for (uint8_t i = 0; i < insn->detail->riscv.op_count; i++) {
      if (insn->detail->riscv.operands[i].access & CS_AC_READ && insn->detail->riscv.operands[i].type == RISCV_OP_REG) {
        *source_reg_valp = regfile[cpu].read(insn->detail->riscv.operands[i].reg);
        source_reg_valp++;
      }
    }

    for (uint8_t i = 0; i < insn->detail->riscv.op_count; i++) {
      if (insn->detail->riscv.operands[i].access & CS_AC_READ && insn->detail->riscv.operands[i].type == RISCV_OP_IMM) {
        *source_reg_valp = insn->detail->riscv.operands[i].imm;
        source_reg_valp++;
      }
    }

    if (insn->detail->regs_write_count) {
      regfile[cpu].write(insn->detail->regs_write[0], ret_val);
    } else {
      for (uint8_t i = 0; i < insn->detail->riscv.op_count; i++) {
        if (insn->detail->riscv.operands[i].access & CS_AC_WRITE && insn->detail->riscv.operands[i].type == RISCV_OP_REG) {
          regfile[cpu].write(insn->detail->riscv.operands[i].reg, ret_val);
          break;
        }
      }
    }

    cs_free(insn, 1);

#ifdef PRINT_TRACE_INFO
    prinf_info();
#endif
    if (instr.ip == PRINT_TRACE_IP)
      prinf_info();

#ifdef COLLECT_INFO
    collect_info(instr);
#endif
  }

  void collect_info(riscv_instr instr)
  {
    total_inst_num++;
    bool is_load = idm_op_is_load(op);
    uint8_t cur_load_type = LOAD_TYPE_NONE;

    if (is_load) {
      total_load_num++;

      uint64_t pc = instr.ip;
      uint64_t vaddr = instr.source_memory[0];
      uint64_t res = instr.ret_val;

      std::unordered_map<uint64_t, load_info_t>::iterator hit_item;
      hit_item = pc_info.find(pc);
      if (hit_item != pc_info.end()) {
        uint64_t old_vaddr = hit_item->second.last_vaddr;
        int64_t old_stride = hit_item->second.stride;
        uint64_t old_stride_conf = hit_item->second.stride_conf;
        uint64_t old_res = hit_item->second.last_res;
        uint8_t old_pt_conf = hit_item->second.pt_conf;
        int64_t old_pt_offset = hit_item->second.pt_offset;

        // Update
        hit_item->second.last_vaddr = vaddr;
        hit_item->second.last_res = res;
        hit_item->second.total_count += 1;

        // Stride
        int64_t stride = vaddr - old_vaddr;
        if (stride == 0) {
          hit_item->second.stride_conf = old_stride_conf;
        } else if (old_stride_conf == 1) {
          hit_item->second.stride_conf = 2;
        } else if (stride == old_stride) {
          if (old_stride_conf < 3)
            hit_item->second.stride_conf += 1;
        } else {
          if (old_stride_conf > 0)
            hit_item->second.stride_conf -= 1;
        }

        if (old_stride_conf == 1)
          hit_item->second.stride = stride;

        if (old_stride_conf > 1 && stride == old_stride && stride != 0) { // TODO: conf2 && stride==old_stride || conf3
          hit_item->second.stride_count += 1;

          // TODO:
          cur_load_type = LOAD_TYPE_ORIGINAL_STRIDE;
        }

        // Pointer Chain
        int64_t pt_offset = vaddr - old_res;
        bool same_addr = vaddr == old_vaddr; // TO aviod the same vaddr & the same ret_val

        if (same_addr) {
          hit_item->second.pt_conf = hit_item->second.pt_conf;
        } else if (old_pt_conf == 1) {
          hit_item->second.pt_conf = 2;
        } else if (pt_offset == old_pt_offset) {
          if (old_pt_conf < 3)
            hit_item->second.pt_conf += 1;
        } else {
          if (old_pt_conf > 0) {
            hit_item->second.pt_conf -= 1;
          }
        }

        if (old_pt_conf == 1)
          hit_item->second.pt_offset = pt_offset;

        if (old_pt_conf > 1 && pt_offset == old_pt_offset && !same_addr) { // TODO: conf2 && offset==old_offset || conf3
          hit_item->second.pt_count += 1;

          // TODO:
          cur_load_type = LOAD_TYPE_ORIGINAL_PT_CHAIN;
        }
      } else { // First Encounter
        load_info_t new_info = {};
        new_info.total_count = 1;
        new_info.last_vaddr = vaddr;
        new_info.stride = 0;
        new_info.stride_conf = 1;
        new_info.stride_count = 0;
        new_info.pt_conf = 1;
        new_info.pt_offset = 0;
        new_info.last_res = res;
        new_info.pt_count = 0;
        new_info.dma_count = 0;
        pc_info.insert(std::pair<uint64_t, load_info_t>(pc, new_info));
      }
    }

    if (instr.ip == PRINT_TRACE_IP)
      std::cout << "Info of pc:" << PRINT_TRACE_IP << "\n total_count : " << pc_info[instr.ip].total_count
                << "\n stride_conf : " << +pc_info[instr.ip].stride_conf << "\n stride_count: " << pc_info[instr.ip].stride_count
                << "\n pt_conf     : " << +pc_info[instr.ip].pt_conf << "\n pt_count    : " << pc_info[instr.ip].pt_count << "\n"
                << std::endl;

    if (cur_load_type && instr.destination_registers[0]) {
      regfile_load_type[instr.destination_registers[0]] = cur_load_type;
      regfile_op[instr.destination_registers[0]] = op;
    } else {
      bool contain = false;
      for (int i = 0; i < NUM_INSTR_SOURCES_RISCV; i++) {
        if (regfile_load_type[instr.source_registers[i]] != LOAD_TYPE_NONE && instr.source_registers[i]) {
          contain = true;
          uint8_t type = LOAD_TYPE_NONE;
          if (regfile_load_type[instr.source_registers[i]] == LOAD_TYPE_ORIGINAL_STRIDE) {
            type = LOAD_TYPE_INFECTED_STRIDE;
          } else if (regfile_load_type[instr.source_registers[i]] == LOAD_TYPE_ORIGINAL_PT_CHAIN) {
            type = LOAD_TYPE_INFECTED_PT_CHAIN;
          } else {
            type = regfile_load_type[instr.source_registers[i]];
          }
          regfile_load_type[instr.destination_registers[0]] = type;
          regfile_op[instr.destination_registers[0]] = op;
        }
      }
      if (!contain && instr.destination_registers[0])
        regfile_load_type[instr.destination_registers[0]] = LOAD_TYPE_NONE;
      if (is_load && contain)
        pc_info[instr.ip].dma_count++;
    }

    return;
  }

  std::size_t num_mem_ops() const { return std::size(destination_memory) + std::size(source_memory); }

  static bool program_order(const ooo_model_instr& lhs, const ooo_model_instr& rhs) { return lhs.instr_id < rhs.instr_id; }
};

#endif
