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

#ifndef CHANNEL_H
#define CHANNEL_H

#include <any>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <string_view>
#include <vector>

#include "access_type.h"
#include "address.h"
#include "champsim.h"

namespace champsim
{

struct cache_queue_stats {
  uint64_t RQ_ACCESS = 0;
  uint64_t RQ_FULL = 0;
  uint64_t RQ_TO_CACHE = 0;
  uint64_t PQ_ACCESS = 0;
  uint64_t PQ_FULL = 0;
  uint64_t PQ_TO_CACHE = 0;
  uint64_t WQ_ACCESS = 0;
  uint64_t WQ_FULL = 0;
  uint64_t WQ_TO_CACHE = 0;
};

class channel
{
  struct request {
    bool is_translated = true;
    bool response_requested = true;

    uint8_t asid[2] = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};
    access_type type{access_type::LOAD};

    uint32_t pf_metadata = 0;
    uint32_t cpu = std::numeric_limits<uint32_t>::max();

    uint64_t id = UINT64_MAX;
    champsim::address address{};
    champsim::address v_address{};
    champsim::address data{};
    uint64_t instr_id = 0;
    champsim::address ip{};

    std::any token{};
  };

  struct response {
    uint64_t id;
    champsim::address address{};
    champsim::address v_address{};
    champsim::address data{};
    uint32_t pf_metadata = 0;
    std::any token{};

    response(uint64_t id_, champsim::address addr, champsim::address v_addr, champsim::address data_, uint32_t pf_meta, std::any _token)
        : id(id_), address(addr), v_address(v_addr), data(data_), pf_metadata(pf_meta), token(_token)
    {
    }
    explicit response(request req) : response(req.id, req.address, req.v_address, req.data, req.pf_metadata, req.token) {}
  };

  template <typename R>
  bool do_add_queue(R& queue, std::size_t queue_size, typename R::value_type& packet);

  std::size_t RQ_SIZE;
  std::size_t PQ_SIZE;
  std::size_t WQ_SIZE;
  champsim::data::bits OFFSET_BITS{};
  bool match_offset_bits = false;

public:
  using response_type = response;
  using request_type = request;
  using stats_type = cache_queue_stats;

  uint64_t& num_reqs;
  std::string LL_NAME;
  std::deque<request_type> RQ{}, PQ{}, WQ{};
  std::deque<response_type> returned{};

  stats_type sim_stats{}, roi_stats{};

  channel(uint64_t& num_reqs, std::string ll_name, std::size_t rq_size, std::size_t pq_size, std::size_t wq_size, champsim::data::bits offset_bits,
          bool match_offset);

  bool add_rq(request_type& packet);
  bool add_wq(request_type& packet);
  bool add_pq(request_type& packet);

  [[nodiscard]] std::size_t rq_occupancy() const;
  [[nodiscard]] std::size_t wq_occupancy() const;
  [[nodiscard]] std::size_t pq_occupancy() const;

  [[nodiscard]] std::size_t rq_size() const;
  [[nodiscard]] std::size_t wq_size() const;
  [[nodiscard]] std::size_t pq_size() const;
};
} // namespace champsim

#endif
