/*
 *    Copyright 2025 The ChampSim Contributors
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

#ifndef KANATA_H
#define KANATA_H

#include <deque>
#include <functional>
#include <optional>
#include <CLI/CLI.hpp>
#include <fmt/os.h>
#include <fmt/ostream.h>

#include "cache.h"
#include "dram_controller.h"
#include "events.h"
#include "instruction.h"
#include "ooo_cpu.h"
#include "ptw.h"

namespace champsim
{
namespace kanata
{
class Kanata final
{
public:
  static constexpr auto cli_key = "Kanata";

  void cli(CLI::App& app)
  {
    app.add_option("--kanata", file_name, "The name of the file to receive Kanata output");
    app.add_option("--kanata-max", max, "The maximum number of instructions for Kanata logging");
    app.add_option("--kanata-skip", skip, "The number of instructions to skip before starting Kanata logging");
  }

  template <Event e, typename... Args>
  inline void handle_event(Args&&... args);

private:
  struct Instr final {
    Kanata& kanata;
    const uint64_t id;
    const bool flushed;
    uint8_t num_lanes = 1;
    size_t num_pending_stores;

    Instr(Kanata& kanata_, uint64_t id_, bool flushed_, size_t num_pending_stores_)
        : kanata(kanata_), id(id_), flushed(flushed_), num_pending_stores(num_pending_stores_)
    {
    }

    ~Instr();
    void start(uint8_t lane, const std::string& stage);
  };

  template <typename T>
  class Queue final
  {
  public:
    T& emplace(uint64_t index)
    {
      if (storage.empty())
        begin = index;
      else
        assert(index - begin >= storage.size());

      storage.resize(index - begin + 1);

      return storage[index - begin];
    }

    T* find(uint64_t index) { return index >= begin && index - begin < storage.size() ? &storage[index - begin] : nullptr; }

    T& front() { return storage.front(); }

    bool empty() const { return storage.empty(); }

    void pop()
    {
      storage.pop_front();
      ++begin;
    }

  private:
    uint64_t begin = UINT64_MAX;
    std::deque<T> storage;
  };

  struct Request final {
    std::shared_ptr<Instr> instr;
    std::string stage;
    uint8_t lane;
    bool forked;

    void start(std::string local_stage);
    void retire();
    void retire(std::string local_stage);
  };

  template <typename ID, typename Label>
  std::shared_ptr<Instr> init(uint32_t cpu, ID id, Label label, bool flushed, size_t num_stores)
  {
    auto index = num;

    ++num;

    if (index < skip || index - skip > max)
      return {};

    if (!file) {
      if (!file_name) {
        skip = UINT64_MAX;
        return {};
      }

      file.emplace(fmt::output_file(*file_name));
      file->print("Kanata\t0004\nC=\t{}\n", current_cycle);
    }

    auto instr = std::make_shared<Instr>(*this, num - skip, flushed, num_stores);
    file->print("I\t{}\t{}\t{}\nL\t{}\t0\t{}\n", instr->id, id, cpu, instr->id, label);
    return instr;
  }

  void complete_cache(const CACHE& cache, uint64_t id, char result)
  {
    auto req = reqs.find(id);
    if (!req)
      return;

    if (!req->forked)
      req->start(cache.LOCAL_NAME + result);

    req->retire();
  }

  void dispatch(const channel& lower, const channel::request_type& req, const std::shared_ptr<Instr>& instr);
  void dispatch(const channel& lower, const channel::request_type& req, const std::shared_ptr<Instr>& instr, uint8_t lane, bool forked = false);

  void fork(const channel& lower, uint64_t id, const channel::request_type& req)
  {
    auto kanata = reqs.find(id);
    if (!kanata)
      return;

    dispatch(lower, req, kanata->instr);
  }

  void forward(const channel& lower, uint64_t id, const channel::request_type& req)
  {
    auto kanata = reqs.find(id);
    if (!kanata)
      return;

    dispatch(lower, req, kanata->instr, kanata->lane);
  }

  void init(const O3_CPU& cpu, const ooo_model_instr& o3)
  {
    auto instr = init(cpu.cpu, o3.instr_id, fmt::streamed(o3), false, o3.destination_memory.size());
    if (!instr)
      return;

    if (cpu.cpu >= cpus.size())
      cpus.resize(cpu.cpu + 1);

    auto& emplaced = cpus[cpu.cpu].emplace(o3.instr_id);
    emplaced.swap(instr);
    emplaced->start(0, "F");
  }

  void retire();

  void retire(uint32_t cpu, std::shared_ptr<Instr>& instr)
  {
    instr.reset();

    while (!cpus[cpu].empty() && !cpus[cpu].front())
      cpus[cpu].pop();

    retire();
  }

  void start(const CACHE& cache, const CACHE::tag_lookup_type& lookup, char stage)
  {
    auto kanata = reqs.find(lookup.id);
    if (!kanata)
      return;

    kanata->start(cache.LOCAL_NAME + stage);
  }

  void start(const O3_CPU& cpu, const ooo_model_instr& o3, const std::string& stage)
  {
    if (cpu.cpu >= cpus.size())
      return;

    auto kanata = cpus[cpu.cpu].find(o3.instr_id);
    if (!kanata)
      return;

    (*kanata)->start(0, stage);
  }

  void start(uint64_t id, const std::string& stage)
  {
    auto kanata = reqs.find(id);
    if (!kanata)
      return;

    kanata->start(stage);
  }

  std::optional<fmt::ostream> file;
  std::optional<std::string> file_name;
  std::vector<Queue<std::shared_ptr<Instr>>> cpus;
  Queue<Request> reqs;
  uint64_t current_cycle = 0;
  uint64_t max = UINT64_MAX;
  uint64_t num = 0;
  uint64_t skip = 0;

  template <Event e, typename... Args>
  friend void handle_event(Kanata& kanata, const Args&... args);
};

template <Event e, typename... Args>
inline void handle_event([[maybe_unused]] Kanata& kanata, [[maybe_unused]] const Args&... args)
{
}

template <>
inline void handle_event<Event::CACHE_EXEC>(Kanata& kanata, const CACHE& cache, const CACHE::tag_lookup_type& lookup)
{
  kanata.start(cache, lookup, 'x');
}

template <>
inline void handle_event<Event::CACHE_FILL>(Kanata& kanata, const CACHE& cache, const CACHE::fill_type& fill)
{
  for (auto req : fill.reqs)
    kanata.complete_cache(cache, req.id, 'c');
}

template <>
inline void handle_event<Event::CACHE_FORWARD>(Kanata& kanata, const CACHE& cache, const CACHE::fill_type& fill, const channel::request_type& req)
{
  kanata.forward(*cache.lower_level, fill.reqs.front().id, req);
}

template <>
inline void handle_event<Event::CACHE_HIT>(Kanata& kanata, const CACHE& cache, const CACHE::tag_lookup_type& lookup)
{
  kanata.complete_cache(cache, lookup.id, 'c');
}

template <>
inline void handle_event<Event::CACHE_MERGE>(Kanata& kanata, const CACHE& cache, const CACHE::tag_lookup_type& lookup)
{
  kanata.start(cache, lookup, 'm');
}

template <>
inline void handle_event<Event::CACHE_PREFETCH>(Kanata& kanata, const channel::request_type& req, const bool& fill_this_level)
{
  auto instr = kanata.init(req.cpu, "", fmt::format("Prefetch [{}]{}", req.address, fill_this_level ? " (fill)" : ""), true, 0);
  if (!instr)
    return;

  auto& kanata_req = kanata.reqs.emplace(req.id);
  kanata_req.instr = std::move(instr);
  kanata_req.lane = 0;
  kanata_req.start("Ds");
}

template <>
inline void handle_event<Event::CACHE_TRANSLATE>(Kanata& kanata, const CACHE& cache, const CACHE::tag_lookup_type& lookup, const channel::request_type& req)
{
  kanata.fork(*cache.lower_translate, lookup.id, req);
}

template <>
inline void handle_event<Event::CACHE_WRITE>(Kanata& kanata, const CACHE& cache, const CACHE::fill_type& fill)
{
  kanata.start(fill.reqs.front().id, cache.LOCAL_NAME + 'w');
}

template <>
inline void handle_event<Event::CACHE_WRITEBACK>(Kanata& kanata, const CACHE& cache, const CACHE::fill_type& fill, const channel::request_type& req)
{
  kanata.fork(*cache.lower_level, fill.reqs.front().id, req);
}

template <>
inline void handle_event<Event::CHANNEL_RESPONSE>(Kanata& kanata, const channel::response_type& response)
{
  auto req = kanata.reqs.find(response.id);
  if (!req)
    return;

  req->retire();
}

template <>
inline void handle_event<Event::COMPLETE>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "C");
}

template <>
inline void handle_event<Event::CYCLE>(Kanata& kanata)
{
  ++kanata.current_cycle;

  if (kanata.file)
    kanata.file->print("C\t1\n");
}

template <>
inline void handle_event<Event::DECODE>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "Dc");
}

template <>
inline void handle_event<Event::DIB_HIT>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "Dh");
}

template <>
inline void handle_event<Event::DISPATCH>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "Ds");
}

template <>
inline void handle_event<Event::DRAM_COMPLETE>(Kanata& kanata, const DRAM_CHANNEL::status_type& status)
{
  for (auto& req : status.reqs) {
    auto kanata_req = kanata.reqs.find(req.id);
    if (!kanata_req || !kanata_req->instr)
      continue;

    if (!kanata_req->forked)
      kanata_req->start("DRAMc");

    kanata_req->retire();
  }
}

template <>
inline void handle_event<Event::DRAM_DISPATCH>(Kanata& kanata, const DRAM_CHANNEL::status_type& status)
{
  kanata.start(status.reqs.front().id, "DRAMds");
}

template <>
inline void handle_event<Event::DRAM_EXEC>(Kanata& kanata, const DRAM_CHANNEL::status_type& status)
{
  kanata.start(status.reqs.front().id, "DRAMx");
}

template <>
inline void handle_event<Event::DRAM_ISSUE>(Kanata& kanata, const DRAM_CHANNEL::status_type& status)
{
  kanata.start(status.reqs.front().id, "DRAMi");
}

template <>
inline void handle_event<Event::DRAM_MERGE>(Kanata& kanata, const DRAM_CHANNEL::status_type& status)
{
  kanata.start(status.reqs.front().id, "DRAMm");
}

template <>
inline void handle_event<Event::EXEC>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "X");
}

template <>
inline void handle_event<Event::FETCH>(Kanata& kanata, const channel& channel, const channel::request_type& req)
{
  if (req.cpu >= kanata.cpus.size())
    return;

  auto instr = kanata.cpus[req.cpu].find(req.instr_id);
  if (!instr)
    return;

  kanata.dispatch(channel, req, *instr, 0);
}

template <>
inline void handle_event<Event::INIT>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.init(cpu, instr);
}

template <>
inline void handle_event<Event::ISSUE>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  if (cpu.cpu >= kanata.cpus.size())
    return;

  auto kanata_instr = kanata.cpus[cpu.cpu].find(instr.instr_id);
  if (!kanata_instr)
    return;

  for (auto& source : instr.source_registers) {
    auto producer = kanata.cpus[cpu.cpu].find(cpu.reg_allocator.get_physical_register(source).producing_instruction_id);
    if (!producer)
      continue;

    kanata.file->print("W\t{}\t{}\t0\n", (*kanata_instr)->id, (*producer)->id);
  }

  (*kanata_instr)->start(0, "I");
}

template <>
inline void handle_event<Event::LOAD>(Kanata& kanata, const channel& channel, const channel::request_type& req)
{
  if (req.cpu >= kanata.cpus.size())
    return;

  auto instr = kanata.cpus[req.cpu].find(req.instr_id);
  if (!instr)
    return;

  kanata.dispatch(channel, req, *instr);
}

template <>
inline void handle_event<Event::PTW_COMPLETE>(Kanata& kanata, const PageTableWalker::mshr_type& mshr)
{
  kanata.start(mshr.id, "PTWc");
}

template <>
inline void handle_event<Event::PTW_STEP>(Kanata& kanata, const PageTableWalker& ptw, const PageTableWalker::mshr_type& mshr, const channel::request_type& req)
{
  kanata.forward(ptw.lower_level(), mshr.id, req);
}

template <>
inline void handle_event<Event::RENAME>(Kanata& kanata, const O3_CPU& cpu, const ooo_model_instr& instr)
{
  kanata.start(cpu, instr, "R");
}

template <>
inline void handle_event<Event::RETIRE>(Kanata& kanata, [[maybe_unused]] const uint32_t& cpu, const std::deque<ooo_model_instr>::const_iterator& begin,
                                        const std::deque<ooo_model_instr>::const_iterator& end, [[maybe_unused]] const uint64_t& current_cycles)
{
  if (cpu >= kanata.cpus.size())
    return;

  for (auto it = begin; it != end; ++it) {
    auto instr = kanata.cpus[cpu].find(it->instr_id);
    if (!instr)
      continue;

    (*instr)->start(0, "W");

    if (!(*instr)->num_pending_stores)
      kanata.retire(cpu, *instr);
  }
}

template <>
inline void handle_event<Event::STORE>(Kanata& kanata, const channel& channel, const channel::request_type& req)
{
  if (req.cpu >= kanata.cpus.size())
    return;

  auto instr = kanata.cpus[req.cpu].find(req.instr_id);
  if (!instr)
    return;

  kanata.dispatch(channel, req, *instr);

  if (!--(*instr)->num_pending_stores)
    kanata.retire(req.cpu, *instr);
}

template <Event e, typename... Args>
inline void Kanata::handle_event(Args&&... args)
{
  kanata::handle_event<e>(*this, std::forward<Args>(args)...);
}
} // namespace kanata
} // namespace champsim

#endif
