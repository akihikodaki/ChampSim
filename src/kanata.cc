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

#include "listeners/kanata.h"

champsim::kanata::Kanata::Instr::~Instr() { kanata.file->print("R\t{}\t{}\t{}\n", id, id, '0' + flushed); }

void champsim::kanata::Kanata::dispatch(const channel& lower, const channel::request_type& req, const std::shared_ptr<Instr>& instr)
{
  if (!instr)
    return;

  dispatch(lower, req, instr, instr->num_lanes++, true);
}

void champsim::kanata::Kanata::dispatch(const channel& lower, const channel::request_type& req, const std::shared_ptr<Instr>& instr, uint8_t lane, bool forked)
{
  if (!instr)
    return;

  auto& kanata = reqs.emplace(req.id);
  kanata.instr = std::move(instr);
  kanata.lane = lane;
  kanata.forked = forked;
  kanata.start(lower.LL_NAME + "ds");
}

void champsim::kanata::Kanata::retire()
{
  if (num - skip < max || !reqs.empty())
    return;

  for (auto& kanata : cpus)
    if (!kanata.empty())
      return;

  file.reset();
}

void champsim::kanata::Kanata::Request::retire()
{
  if (!instr)
    return;

  if (forked)
    instr->kanata.file->print("E\t{}\t{}\t{}\n", instr->id, lane, stage);

  auto& kanata = instr->kanata;
  instr.reset();

  while (!kanata.reqs.empty() && !kanata.reqs.front().instr)
    kanata.reqs.pop();

  kanata.retire();
}

void champsim::kanata::Kanata::Request::start(std::string local_stage)
{
  if (!instr)
    return;

  stage = local_stage;
  instr->start(lane, stage);
}

void champsim::kanata::Kanata::Instr::start(uint8_t lane, const std::string& stage) { kanata.file->print("S\t{}\t{}\t{}\n", id, lane, stage); }
