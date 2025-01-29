#ifndef PREFETCHER_TYCHE_H
#define PREFETCHER_TYCHE_H

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <unordered_map>

#include "cache.h"
#include "chrono.h"
#include "instruction.h"
#include "modules.h"
#include "prefetch.h"
#include "trace_instruction.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-parameter"

extern uint64_t decode_inst_num;
extern uint64_t decode_load_num;
extern uint64_t dct_hit_needbyload_num;
extern uint64_t dct_hit_same_src_num;
extern uint64_t dct_hit_no_depend_num;
extern uint64_t dct_hit_num;
extern uint64_t dct_hit_useless_num;

extern uint64_t dct_search_num;
extern uint64_t dct_write_num;

#define nPRINT_SEARCH_INFECT_INFO
#define nPRINT_INFECT_INFO
#define nPRINT_DCT

#define DEBUG
#define PRINT_INSTR_PC 0xffffffffffffffff
#define PRINT_OPERATE_TRACE XF_DEBUG
#define PRINT_CYCLE_OPERATE XF_DEBUG
#define PRINT_CACHE_FILL_TRACE XF_DEBUG
#define nCOLLECT_LOAD_INFO
#define COLLECT_LOAD_PC 0xffffffffffffffff

#define AMEND_OFF

extern bool start_print;

#ifdef PREF_NUM
std::map<uint64_t, uint64_t> pref_num;
#endif

extern uint8_t trace_type;

extern uint64_t total_exc_num;

extern uint64_t isq_search_num;

bool only_stride = false;

typedef struct load_miss_info {
  uint64_t total_count;
  uint64_t miss_count;
} load_miss_info_t;
unordered_map<uint64_t, load_miss_info_t> load_info;
unordered_map<uint64_t, uint64_t> load_addr_info;

#ifdef IC_LENGTH_INFO
extern unordered_map<uint64_t, uint64_t> ic_length_info;
extern unordered_map<IDM_OP, uint64_t> ict_op_info;
#endif

class tyche : public champsim::modules::prefetcher
{
private:
  champsim::chrono::clock::time_point& current_time;
  pt_format_t pt[256] = {0, 0};

#if (defined DECODE_COLLECT_INFO) || (defined MISS_COLLECT_INFO)
  uint8_t complex_infect_info[256] = {0};
#endif

  IPT_L1 ipt[IPT_NUM];
  DCT dct; // instruction chain table
  AGQ agq;

  MEMORY_DATA mem_data;

  // Performance Counters
  uint64_t idm_double_stride = 0; // load which both sources are dependent on stride/pt chain

  void search_ima(uint64_t ip, uint64_t pf_address, uint64_t origin_addr, int64_t delta)
  {
    int64_t dct_ptr = dct.search_pc(ip);

    if (dct_ptr != -1 && dct.has_successor(dct_ptr) && dct.buffer[dct_ptr].head) {
      AGQ_ITEM new_agq_item;
      new_agq_item.valid = true;
      new_agq_item.issued = true;
      new_agq_item.ret_value = pf_address;
      new_agq_item.dct_ptr = dct_ptr;
      new_agq_item.is_load = true;
#ifdef PREFETCH_DEBUG
      new_agq_item.time = current_time;
#endif
      agq.insert(new_agq_item);
    }

#ifdef IC_LENGTH_INFO
    if (dct_ptr != -1 && dct.has_successor(dct_ptr, true) && dct.buffer[dct_ptr].head) {
      vector<uint64_t> idx_vec = {(uint64_t)dct_ptr};

      int cnt = 0;
      while (!idx_vec.empty()) {
        uint64_t index = *idx_vec.begin();
        idx_vec.erase(idx_vec.begin());

        for (auto it = dct.buffer.begin(); it != dct.buffer.end(); it++) {
          if (it->valid && !it->head && it->last_dct_ptr == index && it->need_handle()) {
            idx_vec.push_back(distance(dct.buffer.begin(), it));
            ict_op_info[it->op]++;
          }
        }
        cnt++;

        if (cnt > 10000) {
          cout << "dct_ptr: " << dct_ptr << endl;
          dct.print_buffer();
          exit(1);
        }
      }

      std::unordered_map<uint64_t, uint64_t>::iterator hit_item;
      hit_item = ic_length_info.find(cnt);
      if (hit_item != ic_length_info.end()) {
        ic_length_info[cnt]++;
      } else {
        ic_length_info.insert(pair<uint64_t, uint64_t>(cnt, 1));
      }
    }

#endif
  }

  uint8_t update_conf(int64_t stride, int64_t last_stride, uint8_t conf)
  {
    uint8_t conf_ret;
    if (stride == 0) {
      conf_ret = conf;
    } else if (conf == 1) {
      conf_ret = conf + 1;
    } else if (stride == last_stride) {
      conf_ret = (conf < SP_CONF_MAX) ? (conf + 1) : conf;
    } else {
      conf_ret = (conf > 0) ? (conf - 1) : 0;
    }

    return conf_ret;
  }

public:
  tyche(CACHE* cache) : champsim::modules::prefetcher(cache), current_time(cache->current_time)
  {
    for (uint32_t i = 0; i < IPT_NUM; i++) {
      ipt[i].conf = 0;
      ipt[i].rplc_bits = i;
    }

    agq.SIZE = AGQ_SIZE;
    agq.pop_when_full = false;
#ifdef PREFETCH_DEBUG
    agq.name = "l1_agq";
#endif

    cout << "Stride ISQ Size: " << AGQ_SIZE << endl;

    cout << "L1 Stride distance: " << L1_STRIDE_DISTANCE << endl;

#ifdef IC_LENGTH_INFO
    for (uint64_t i = IDM_INVALID; i < IDM_LD_D; i++) {
      ict_op_info.insert(pair<IDM_OP, uint64_t>((IDM_OP)i, 0));
    }
#endif
  }

  void prefetcher_cycle_operate()
  {
#ifdef PREFETCH_DEBUG
    agq.check_state(dct);
    dct.check_state();
#endif

    auto agq_item = agq.first_ready_item();
    if (agq_item != agq.buffer.end()) {
      uint64_t dct_ptr = agq_item->dct_ptr;
      auto dct_item = dct.buffer.begin() + dct_ptr;

      isq_search_num++;

#if PRINT_CYCLE_OPERATE == 1
      // if(start_print)
      cout << "[cycle_operate    ] ";
      agq_item->print();
#endif
      if (idm_op_is_load(dct_item->op)) {
        bool has_successor = dct.has_successor(dct_ptr);
        uint64_t pf_address = dct_item->src + agq_item->ret_value;
        if (pf_address & 0xffffff0000000000 || pf_address == 0) {
          agq.AGQ_BEYOND_NUM++;
          agq.buffer.erase(agq_item);
        } else {
          int succ = prefetch_line(pf_address, true, 0);

#ifdef PREF_NUM
          auto hit_item = pref_num.find(dct_item->ip);
          if (hit_item != pref_num.end()) {
            hit_item->second++;
          } else {
            pref_num.insert(std::pair<uint64_t, uint64_t>(dct_item->ip, 1));
          }
#endif

          if (succ == 1) {
            agq_item->issued = true;
            agq_item->ret_value = pf_address;
            if (!has_successor) {
              agq.buffer.erase(agq_item);
            }
          }
          total_exc_num++;
        }
      } else {
        uint64_t alu_ret_val = dct_item->execute_alu(agq_item->ret_value);
        if (idm_op_is_mul(dct_item->op)) {
          agq_item->issued = true;
          agq_item->alu_res = alu_ret_val;
          agq_item->alu_cycle = 3;
        } else {
          agq_item->issued = true;
          agq_item->alu_res = alu_ret_val;
          agq_item->alu_cycle = 2;
        }
        total_exc_num++;
      }
    }

    for (auto it = agq.buffer.begin(); it != agq.buffer.end(); it++) {
      auto dct_item = dct.buffer.begin() + it->dct_ptr;
      if (!idm_op_is_load(dct_item->op) && it->issued && it->alu_cycle > 0) {
        it->alu_cycle--;
      }
    }

    for (auto it = agq.buffer.begin(); it != agq.buffer.end(); it++) {
      auto dct_item = dct.buffer.begin() + it->dct_ptr;

      if (!idm_op_is_load(dct_item->op) && it->issued && it->alu_cycle == 0) {
        bool has_successor = agq.update_src(dct, it, it->alu_res);
        if (!has_successor) {
          agq.buffer.erase(it);
          agq.AGQ_ALU_ALONE++;
        }
        break;
      }
    }
  }

  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip, uint8_t cache_hit, bool useful_prefetch, access_type type,
                                    uint32_t metadata_in)
  {
    auto addr64 = addr.to<uint64_t>();
    auto ip64 = ip.to<uint64_t>();
    uint64_t trace_ip = (trace_type == TRACE_TYPE_RISCV) ? (ip64 >> 2) : ip64;
    uint32_t hit_idx = IPT_NUM;

#ifdef PREFETCH_DEBUG
    bool dbg_hit = false;
#endif

    //// Find hit item
    for (uint32_t i = 0; i < IPT_NUM; i++) {
      if (ipt[i].conf != 0 && ipt[i].ip == trace_ip) {
        hit_idx = i;

#ifdef PREFETCH_DEBUG
        if (dbg_hit) {
          cout << "Multi Way Hit!!!" << endl;
          exit(1);
        }
        dbg_hit = true;
#endif
      }
    }

    //// Stride Hit
    if (hit_idx != IPT_NUM) {
      IPT_L1 ipt_hit_item = ipt[hit_idx];

      int64_t new_stride = addr64 - ipt_hit_item.last_addr;

      bool ignore = new_stride == 0 /*&& !younger*/;

      bool conf_trigger =
          ipt_hit_item.conf > 1 && ipt_hit_item.stride != 0 && ((ipt_hit_item.stride == new_stride) || (ipt_hit_item.conf >= 3 && new_stride != 0));
      bool trigger_prefetch = conf_trigger; //|| change_to_use_line;

      if (trigger_prefetch) {
        int64_t stride = ipt_hit_item.stride;
        uint64_t distance = L1_STRIDE_DISTANCE; // 32
        int64_t delta = stride * distance;

        uint64_t pf_address = addr64 + delta;
        int succ = prefetch_line(pf_address, true, 0);

#ifdef PREF_NUM
        auto hit_item = pref_num.find(ip64);
        if (hit_item != pref_num.end()) {
          hit_item->second++;
        } else {
          pref_num.insert(std::pair<uint64_t, uint64_t>(ip64, 1));
        }
#endif

        // Insert into AGQ
        if (!only_stride && succ != 0) {
          search_ima(trace_ip, pf_address, addr64, delta);
        }
      }

      if (!ignore) {
        ipt[hit_idx].last_addr = addr64;
        ipt[hit_idx].conf = update_conf(new_stride, ipt_hit_item.stride, ipt_hit_item.conf);

        if (ipt_hit_item.conf == 1) {
          ipt[hit_idx].stride = new_stride;
        }
      }

      for (uint32_t j = 0; j < IPT_NUM; j++) {
        if (ipt[j].rplc_bits > ipt_hit_item.rplc_bits) {
          ipt[j].rplc_bits--;
        }
      }
      ipt[hit_idx].rplc_bits = IPT_NUM - 1;

      return metadata_in;
    } else {
      // Stride Miss
      uint8_t ip_idx = IPT_NUM;
      uint8_t rplc0_idx = IPT_NUM;
      uint8_t conf0_idx = IPT_NUM;
      uint8_t conf0_rplc = IPT_NUM;

      for (uint32_t i = 0; i < IPT_NUM; i++) {
        if (ipt[i].conf < 2 && ipt[i].rplc_bits < conf0_rplc) {
          conf0_idx = i;
          conf0_rplc = ipt[i].rplc_bits;
        }

        if (ipt[i].ip == trace_ip) {
          ip_idx = i;
        }

        if (ipt[i].rplc_bits == 0) {
          rplc0_idx = i;

#ifdef PREFETCH_DEBUG
          if (dbg_hit) {
            cout << "Find Multiple Entries rplc_bits==0!" << endl;
            exit(1);
          }
          dbg_hit = true;
#endif
        }
      }

      uint8_t rplc_idx = (ip_idx < IPT_NUM) ? ip_idx : (conf0_idx < IPT_NUM) ? conf0_idx : rplc0_idx;

      ipt[rplc_idx].ip = trace_ip;
      ipt[rplc_idx].last_addr = addr64;
      ipt[rplc_idx].conf = 1;

      for (uint32_t j = 0; j < IPT_NUM; j++) {
        if (ipt[j].rplc_bits > ipt[rplc_idx].rplc_bits) {
          ipt[j].rplc_bits--;
        }
      }
      ipt[rplc_idx].rplc_bits = IPT_NUM - 1;
      // ipt[rplc_idx].pref_filter.clear();

#ifdef PREFETCH_DEBUG
      assert(rplc_idx < IPT_NUM);
#endif
    }

    return metadata_in;
  }

  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in)
  {
    auto addr64 = addr.to<uint64_t>();

#if PRINT_CACHE_FILL_TRACE == 1
    cout << "\t[l1d_prefetcher_cache_fill]"
         << " time: " << time << hex << ", addr: " << addr64 << endl;
#endif

    idm_load_return(dct, agq, mem_data, addr64);

    return metadata_in;
  }

  void prefetcher_decode(const ooo_model_instr& instr)
  {
    // Infect
    //// Search propagation table
    int depend_idx = 0xff;
    int undepend_idx = 0xff;
    bool is_depend[2] = {false};
    for (size_t i = 0; i < instr.source_registers.size(); i++) {
      if (pt[instr.source_registers[i]].type != LOAD_TYPE_NONE) {
        is_depend[i] = true;
        depend_idx = i;
      }
    }

    //// Search IPT
    auto ip = instr.ip.to<uint64_t>();
    uint64_t trace_ip = (trace_type == TRACE_TYPE_RISCV) ? (ip >> 2) : ip;
    uint32_t stride_hit_idx = IPT_NUM;

    for (uint32_t i = 0; i < IPT_NUM; i++) {
      if (ipt[i].conf >= 3 && ipt[i].ip == trace_ip)
        stride_hit_idx = i;
    }
    bool is_load = idm_op_is_load(instr.op);
    bool ip_stride_hit = is_load && (stride_hit_idx < IPT_NUM);

    bool has_depend, both_depend;

    if (!instr.destination_registers.empty()) {
      has_depend = is_depend[0] || is_depend[1];
      both_depend = is_depend[0] && is_depend[1];
      undepend_idx = (depend_idx == 0) ? 1 : 0;
      int64_t last_dct_ptr = depend_idx == 0xff ? -1 : pt[instr.source_registers[depend_idx]].dct_ptr;

      //// Search DCT
      int64_t dct_hit_idx = dct.search_pc(trace_ip);
      bool dct_hit = (dct_hit_idx != -1);
      bool prev_dct_hit = (last_dct_ptr != -1) && dct.buffer[last_dct_ptr].valid;

      IDM_OP op = instr.op;
      // if(op == IDM_INVALID)
      //     cout << "NOTE: Unknown op! pc: " << hex << ip << dec << endl;
      if (ip_stride_hit) {
        if (!dct_hit) {
          DCT_ITEM new_item;
          new_item.valid = true;
          new_item.head = true;
          new_item.formed = true;
          new_item.conf = 3;
          new_item.pc = trace_ip;
          new_item.op = op;
          new_item.const_idx = true;
          new_item.src = 0;
          new_item.last_dct_ptr = 0;
#ifdef PREFETCH_DEBUG
          new_item.ip = ip;
          new_item.time = current_time;
#endif
          dct_hit_idx = dct.insert(pt, agq, new_item);

          dct_write_num++;
        } else {
          dct.buffer[dct_hit_idx].head = true;
          dct.buffer[dct_hit_idx].formed = true;
          dct.buffer[dct_hit_idx].conf = 3;
          dct.buffer[dct_hit_idx].last_dct_ptr = 0;
        }

        dct_search_num++;
      }

      if (!ip_stride_hit && has_depend && !both_depend) {
        uint64_t new_src = instr.source_reg_val[undepend_idx];

        if (!dct_hit && prev_dct_hit) {
          DCT_ITEM new_item;
          new_item.valid = true;
          new_item.head = false;
          new_item.formed = is_load;
          new_item.conf = 1;
          new_item.pc = trace_ip;
          new_item.op = op;
          new_item.const_idx = undepend_idx;
          new_item.src = new_src;
          new_item.last_dct_ptr = 1;
#ifdef PREFETCH_DEBUG
          new_item.ip = ip;
          new_item.time = current_time;
#endif
          dct_hit_idx = dct.insert(pt, agq, new_item);

          // Update formed backward
          if (is_load) {
            uint64_t idx = last_dct_ptr;
            while (!dct.buffer[idx].formed) {
              dct.buffer[idx].formed = true;
              idx = dct.buffer[idx].last_dct_ptr;
            }
          }

          dct_write_num++;
        } else if (dct_hit) {
          uint64_t old_src = dct.buffer[dct_hit_idx].src;
          uint8_t old_conf = dct.buffer[dct_hit_idx].conf;
#ifdef COLLECT_SAME_SRC
          if (dct.buffer[dct_hit_idx].formed) {
            dct_hit_needbyload_num++;
            dct_hit_same_src_num += (old_src == new_src);
          }
#endif
#ifdef COLLECT_USELESS
          bool collect_useless_conf_3 = dct.buffer[dct_hit_idx].conf == 3;
          bool collect_useless_needbyload = dct.buffer[dct_hit_idx].formed;
          dct_hit_num += collect_useless_conf_3;
          dct_hit_useless_num += collect_useless_conf_3 && !collect_useless_needbyload;
#endif
          if (old_src == new_src) {
            dct.buffer[dct_hit_idx].conf = (old_conf < 3) ? old_conf + 1 : old_conf;
          } else {
            dct.buffer[dct_hit_idx].conf = (old_conf > 0) ? old_conf - 1 : old_conf;
          }
          if (instr.source_registers[undepend_idx] == 0) {
            dct.buffer[dct_hit_idx].conf = 3;
          }

          /**
           * If a dependent load is trained as a stride errorly, we should restore the correct
           * info. We should reset head & const_idx.
           */
          dct.buffer[dct_hit_idx].head = false;
          dct.buffer[dct_hit_idx].const_idx = undepend_idx;
          dct.buffer[dct_hit_idx].src = new_src;
          dct.buffer[dct_hit_idx].last_dct_ptr = last_dct_ptr;
#ifdef PREFETCH_DEBUG
          dct.buffer[dct_hit_idx].time = current_time;
#endif

          if (old_conf != 3 && old_conf != 0) {
            dct_write_num++;
          }
        }

        dct_search_num++;
      }

      if (is_load && both_depend)
        idm_double_stride++;

      if (ip_stride_hit && dct_hit_idx != -1) {
        pt[instr.destination_registers[0]].type = LOAD_TYPE_ORIGINAL_STRIDE;
        pt[instr.destination_registers[0]].dct_ptr = dct_hit_idx;
      } else if (has_depend && !both_depend && dct_hit_idx != -1) {
        pt[instr.destination_registers[0]].type = LOAD_TYPE_INFECTED_STRIDE;
        pt[instr.destination_registers[0]].dct_ptr = dct_hit_idx;
      } else {
        pt[instr.destination_registers[0]].type = LOAD_TYPE_NONE;
      }

#if (defined DECODE_COLLECT_INFO) || (defined MISS_COLLECT_INFO)
#define DECODE_NONE 0
#define DECODE_INFECT 1
#define DECODE_COMPLEX 2
      uint8_t depend_type[2];
      depend_type[0] = complex_infect_info[instr.source_registers[0]];
      depend_type[1] = complex_infect_info[instr.source_registers[1]];
      bool decode_has_depend = (depend_type[0] != DECODE_NONE) || (depend_type[1] != DECODE_NONE);
      bool decode_both_depend = (depend_type[0] != DECODE_NONE) && (depend_type[1] != DECODE_NONE);
      bool depend_complex = depend_type[0] == DECODE_COMPLEX || depend_type[1] == DECODE_COMPLEX;

      if (ip_stride_hit) {
        complex_infect_info[instr.destination_registers[0]] = DECODE_INFECT;
      } else if (depend_complex || (has_depend && idm_op_is_complex(op))) {
        complex_infect_info[instr.destination_registers[0]] = DECODE_COMPLEX;
      } else if (has_depend) {
        complex_infect_info[instr.destination_registers[0]] = DECODE_INFECT;
      } else {
        complex_infect_info[instr.destination_registers[0]] = LOAD_TYPE_NONE;
      }

#ifdef MISS_COLLECT_INFO
      if (ip_stride_hit) {
        instr.load_type = LOAD_STRIDE;
      } else if (depend_complex || (has_depend && idm_op_is_complex(op))) {
        instr.load_type = LOAD_COMPLEX;
      } else if (has_depend) {
        instr.load_type = LOAD_INFECT;
      } else {
        instr.load_type = LOAD_NONE;
      }
#endif

      std::unordered_map<uint64_t, decode_load_info_t>::iterator hit_item;
      hit_item = decode_pc_info.find(ip);
      if (is_load) {
        if (hit_item != decode_pc_info.end()) {
          hit_item->second.total_count++;
          if (ip_stride_hit) {
            hit_item->second.stride_count++;
          } else if (decode_has_depend) {
            if (!decode_both_depend) {
              hit_item->second.ima_single_count++;
            } else if (decode_both_depend) {
              hit_item->second.ima_double_count++;
            }

            if (depend_complex) {
              hit_item->second.ima_complex_count++;
            }
          }
        } else {
          decode_load_info_t new_info = {
              .total_count = 1,
              .stride_count = 0,
              .ima_single_count = 0,
              .ima_double_count = 0,
          };
          decode_pc_info.insert(std::pair<uint64_t, decode_load_info_t>(ip, new_info));
        }
      }
#endif
    }
#ifdef DECODE_COLLECT_INFO
    decode_inst_num++;
    decode_load_num += is_load;
#endif

#ifdef PRINT_DCT
    if (start_print) {
      cout << "pc: " << hex << ip << dec << ", time: " << current_time << ", ip_stride_hit: " << +ip_stride_hit << endl;

      for (int i = 0; i < 4; i++) {
        cout << "dct[" << i << "]: ";
        dct.buffer[i].print();
      }
    }
#endif
#ifdef PRINT_INFECT_INFO
    if (start_print) {
      cout << "pc: " << hex << ip << dec << ", time: " << current_time << ", ip_stride_hit: " << +ip_stride_hit << endl;

      for (int i = 0; i < 32; i++) {
        cout << "pt[" << i << "]" << hex << ", type : " << +pt[i].type << ", ipt_way : " << +pt[i].ipt_way << ", ipt_set : " << +pt[i].ipt_set << dec << endl;
      }
      cout << endl;
    }
#endif
  }

  void prefetcher_retire(const ooo_model_instr& instr)
  {
    //// Search DCT
    auto ip = instr.ip.to<uint64_t>();
    uint64_t trace_ip = (trace_type == TRACE_TYPE_RISCV) ? (ip >> 2) : ip;
    int64_t dct_hit_idx = dct.search_pc(trace_ip);
    if (dct_hit_idx != -1) {
      if (dct.buffer[dct_hit_idx].head && dct.buffer[dct_hit_idx].cnt == 0) {
        vector<uint64_t> cand = {(uint64_t)dct_hit_idx};
        while (!cand.empty()) {
          uint64_t index = cand.back();
          cand.pop_back();

          for (auto it = dct.buffer.begin(); it != dct.buffer.end(); it++) {
            if (it->valid && !it->head && it->last_dct_ptr == index) {
              if (it->cnt > 115) {
                it->dense = true;
              } else {
                it->dense = false;
              }

              it->cnt = 0;
              cand.push_back(distance(dct.buffer.begin(), it));
            }
          }
        }
      }

      if (dct.buffer[dct_hit_idx].head) {
        dct.buffer[dct_hit_idx].cnt++;
      } else if (dct.buffer[dct_hit_idx].cnt < 255) {
        dct.buffer[dct_hit_idx].cnt++;
      }
    }
  }

  void prefetcher_read(std::string fname)
  {
    mem_data.set_init_fname(fname, false);
    mem_data.init();
  }

  void prefetcher_write(champsim::address addr, uint64_t wdata, uint8_t size) { mem_data.write(addr.to<uint64_t>(), wdata, size); }

  void prefetcher_final_stats()
  {
    cout << "DCT_FULL: " << dct.IDM_ISSSUEQ_FULL << endl;
    cout << "AGQ_FULL: " << agq.AGQ_FULL << endl;

#ifdef PREF_NUM
    for (map<uint64_t, uint64_t>::iterator it = pref_num.begin(); it != pref_num.end(); it++) {
      cout << "ip = " << hex << it->first << dec << ",  num = " << it->second << endl;
    }
#endif

#ifdef COLLECT_LOAD_INFO
    std::vector<uint64_t> load_pc;
    for (auto& [key, value] : load_info)
      load_pc.emplace_back(key);

    sort(load_pc.begin(), load_pc.end(), [&](const uint64_t& a, const uint64_t& b) -> bool {
      return load_info[a].total_count == load_info[b].total_count ? a < b : load_info[a].total_count > load_info[b].total_count;
    });

    uint32_t num = 50;
    uint32_t icount = 0;
    for (auto i = load_pc.begin(); i != load_pc.end(); i++) {
      uint64_t pc = *i;
      load_miss_info_t info_item = load_info[*i];
      double miss_ratio = 1.0 * info_item.miss_count / info_item.total_count;
      printf("[%02d]: PC: %#lx, load num: %8ld, miss num: %8ld, miss ratio: %6.2f%%\n", icount, pc, info_item.total_count, info_item.miss_count,
             miss_ratio * 100);

      // icount++;
      if (++icount == num)
        break;
    }
#endif

#if COLLECT_LOAD_PC != 0xffffffffffffffff
    cout << "The Address Range of PC: " << hex << COLLECT_LOAD_PC << dec;
    cout << " is " << load_addr_info.size() << " * size of the load." << endl;

    std::vector<uint64_t> load_addr;
    for (auto& [key, value] : load_addr_info)
      load_addr.emplace_back(key);

    sort(load_addr.begin(), load_addr.end(),
         [&](const uint64_t& a, const uint64_t& b) -> bool { return load_addr_info[a] == load_addr_info[b] ? a < b : load_addr_info[a] > load_addr_info[b]; });

    uint32_t addr_num = 10;
    uint32_t addr_icount = 0;
    for (auto i = load_addr.begin(); i != load_addr.end(); i++) {
      printf("[%02d]: num: %8ld\n", addr_icount, load_addr_info[*i]);

      // icount++;
      if (++addr_icount == addr_num)
        break;
    }
#endif
  }
};

#pragma GCC diagnostic pop

#endif
