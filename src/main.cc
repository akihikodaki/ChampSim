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

#include <algorithm>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#include "champsim.h"
#include "champsim_constants.h"
#include "core_inst.inc"
#include "phase_info.h"
#include "stats_printer.h"
#include "tracereader.h"
#include "vmem.h"
#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include "arch.h"
#include "memory_data.h"
#include "regfile.h"
#include "prefetch.h"
#include "instruction.h"

uint64_t total_inst_num = 0;
uint64_t total_load_num = 0;
uint64_t decode_inst_num = 0;
uint64_t decode_load_num = 0;
uint64_t miss_load_num = 0;
uint64_t miss_stride_num = 0;
uint64_t miss_ima_num = 0;
uint64_t miss_ima_complex_num = 0;
uint64_t dct_hit_needbyload_num = 0;
uint64_t dct_hit_same_src_num = 0;
uint64_t dct_hit_num = 0;
uint64_t dct_hit_useless_num = 0;
// uint64_t dct_hit_no_depend_num = 0;
// uint64_t miss_ima_single_num = 0;
// uint64_t miss_ima_double_num = 0;
uint64_t dma_consumer_num = 0;
uint64_t dma_caught_num = 0;
uint64_t dma_consumer_inst_num = 0;
uint64_t dma_caught_inst_num = 0;

uint64_t dct_search_num = 0;
uint64_t dct_write_num = 0;

uint64_t isq_search_num = 0;
uint64_t isq_write_num = 0;

std::unordered_map<uint64_t, load_info_t> pc_info;
std::unordered_map<uint64_t, decode_load_info_t> decode_pc_info;
std::unordered_map<uint64_t, uint64_t> ic_length_info;
std::unordered_map<IDM_OP, uint64_t> ict_op_info;
map<uint64_t, uint64_t> consumer_map;
uint64_t total_exc_num = 0;

uint64_t ima_pref_num = 0;
uint64_t ima_pref_miss_num = 0;

uint8_t regfile_load_type[64] = {};
uint8_t regfile_op[64] = {};
MEMORY_DATA mem_data[NUM_CPUS];
REGFILE regfile[NUM_CPUS];
extern AGQ agq[NUM_CPUS];
extern DCT dct[NUM_CPUS];

uint8_t trace_type = TRACE_TYPE_INVALID;
bool compressed_memory = false;

namespace champsim
{
Arch arch;
std::vector<phase_stats> main(environment& env, std::vector<phase_info>& phases, std::vector<tracereader>& traces);
}

int main(int argc, char** argv)
{
  champsim::configured::generated_environment gen_environment{};

  CLI::App app{"A microarchitecture simulator for research and education"};

  bool knob_cloudsuite{false};
  bool knob_riscv{false};
  uint64_t warmup_instructions = 0;
  uint64_t simulation_instructions = std::numeric_limits<uint64_t>::max();
  std::string json_file_name;
  std::vector<std::string> trace_names;

  auto set_heartbeat_callback = [&](auto) {
    for (O3_CPU& cpu : gen_environment.cpu_view())
      cpu.show_heartbeat = false;
  };

  app.add_flag("-c,--cloudsuite", knob_cloudsuite, "Read all traces using the cloudsuite format");
  app.add_flag("-r,--riscv", knob_riscv, "Read all traces using the RISC-V format");
  app.add_flag("--hide-heartbeat", set_heartbeat_callback, "Hide the heartbeat output");
  auto warmup_instr_option = app.add_option("-w,--warmup-instructions", warmup_instructions, "The number of instructions in the warmup phase");
  auto deprec_warmup_instr_option =
      app.add_option("--warmup_instructions", warmup_instructions, "[deprecated] use --warmup-instructions instead")->excludes(warmup_instr_option);
  auto sim_instr_option = app.add_option("-i,--simulation-instructions", simulation_instructions,
                                         "The number of instructions in the detailed phase. If not specified, run to the end of the trace.");
  auto deprec_sim_instr_option =
      app.add_option("--simulation_instructions", simulation_instructions, "[deprecated] use --simulation-instructions instead")->excludes(sim_instr_option);

  auto json_option =
      app.add_option("--json", json_file_name, "The name of the file to receive JSON output. If no name is specified, stdout will be used")->expected(0, 1);

  app.add_option("traces", trace_names, "The paths to the traces")->required()->expected(NUM_CPUS)->check(CLI::ExistingFile);

  CLI11_PARSE(app, argc, argv);

  const bool warmup_given = (warmup_instr_option->count() > 0) || (deprec_warmup_instr_option->count() > 0);
  const bool simulation_given = (sim_instr_option->count() > 0) || (deprec_sim_instr_option->count() > 0);

  if (deprec_warmup_instr_option->count() > 0)
    fmt::print("WARNING: option --warmup_instructions is deprecated. Use --warmup-instructions instead.\n");

  if (deprec_sim_instr_option->count() > 0)
    fmt::print("WARNING: option --simulation_instructions is deprecated. Use --simulation-instructions instead.\n");

  if (simulation_given && !warmup_given)
    warmup_instructions = simulation_instructions * 2 / 10;

  if (knob_cloudsuite && knob_riscv) {
    fmt::print("Both CloudSuite and RISC-V formats are requested. Choose either of them.\n");
    return 1;
  }

  if (knob_cloudsuite) {
    trace_type = TRACE_TYPE_CLOUDSUITE;
  } else if (knob_riscv) {
    trace_type = TRACE_TYPE_RISCV;
  } else {
    trace_type = TRACE_TYPE_X86;
  }

  // Regfile & Memory initializaiton
  if(trace_type==TRACE_TYPE_RISCV){
    uint8_t i = 0;
    for (auto& trace_name : trace_names) {
      // Regfile
      int pos = trace_name.find(".champsim.xz");
      if(pos == -1){
        pos = trace_name.find(".champsim.trace.xz");
      }
      if(pos == -1){
        std::cout << "Trace Name Error!" << std::endl;
      }
      string prefix_name = trace_name.substr(0, pos);

      regfile[i].set_init_fname(prefix_name + ".regfile.txt");
      regfile[i].init();

      // Memory
      mem_data[i].set_init_fname(prefix_name + ".memory.bin", compressed_memory);
      mem_data[i].init();
      i++;
    }

    champsim::arch = { 2, UINT8_MAX, UINT8_MAX };
  } else {
    champsim::arch = {
      champsim::REG_STACK_POINTER,
      champsim::REG_FLAGS,
      champsim::REG_INSTRUCTION_POINTER
    };
  }

  std::vector<champsim::tracereader> traces;
  std::transform(
      std::begin(trace_names), std::end(trace_names), std::back_inserter(traces),
      [repeat = simulation_given, i = uint8_t(0)](auto name) mutable { return get_tracereader(name, i++, trace_type, repeat); });

  std::vector<champsim::phase_info> phases{
      {champsim::phase_info{"Warmup", true, warmup_instructions, std::vector<std::size_t>(std::size(trace_names), 0), trace_names},
       champsim::phase_info{"Simulation", false, simulation_instructions, std::vector<std::size_t>(std::size(trace_names), 0), trace_names}}};

  for (auto& p : phases)
    std::iota(std::begin(p.trace_index), std::end(p.trace_index), 0);

  fmt::print("\n*** ChampSim Multicore Out-of-Order Simulator ***\nWarmup Instructions: {}\nSimulation Instructions: {}\nNumber of CPUs: {}\nPage size: {}\n\n",
             phases.at(0).length, phases.at(1).length, std::size(gen_environment.cpu_view()), PAGE_SIZE);

  auto phase_stats = champsim::main(gen_environment, phases, traces);

  fmt::print("\nChampSim completed all CPUs\n\n");

  champsim::plain_printer{std::cout}.print(phase_stats);

  for (CACHE& cache : gen_environment.cache_view())
    cache.impl_prefetcher_final_stats();

  for (CACHE& cache : gen_environment.cache_view())
    cache.impl_replacement_final_stats();

  if (json_option->count() > 0) {
    if (json_file_name.empty()) {
      champsim::json_printer{std::cout}.print(phase_stats);
    } else {
      std::ofstream json_file{json_file_name};
      champsim::json_printer{json_file}.print(phase_stats);
    }
  }

  return 0;
}
