/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

#ifndef FCL_BROADPHASE_SIMPLEHASHTABLE_INL_H
#define FCL_BROADPHASE_SIMPLEHASHTABLE_INL_H

#include "fcl/broadphase/detail/simple_hash_table.h"

#include <iterator>

namespace fcl
{

namespace detail
{

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
SimpleHashTable<Key, Data, HashFnc>::SimpleHashTable(const HashFnc& h)
  : h_(h)
{
  // Do nothing
}

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
void SimpleHashTable<Key, Data, HashFnc>::init(size_t size)
{
  if(size == 0)
  {
    throw std::logic_error("SimpleHashTable must have non-zero size.");
  }

  table_.resize(size);
  table_size_ = size;
}

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
void SimpleHashTable<Key, Data, HashFnc>::insert(Key key, Data value)
{
  std::vector<unsigned int> indices = h_(key);
  size_t range = table_.size();
  for(size_t i = 0; i < indices.size(); ++i)
    table_[indices[i] % range].push_back(value);
}

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
std::vector<Data> SimpleHashTable<Key, Data, HashFnc>::query(Key key) const
{
  size_t range = table_.size();
  std::vector<unsigned int> indices = h_(key);
  std::set<Data> result;
  for(size_t i = 0; i < indices.size(); ++i)
  {
    unsigned int index = indices[i] % range;
    std::copy(table_[index].begin(), table_[index].end(),
              std::inserter(result, result.end()));
  }

  return std::vector<Data>(result.begin(), result.end());
}

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
void SimpleHashTable<Key, Data, HashFnc>::remove(Key key, Data value)
{
  size_t range = table_.size();
  std::vector<unsigned int> indices = h_(key);
  for(size_t i = 0; i < indices.size(); ++i)
  {
    unsigned int index = indices[i] % range;
    table_[index].remove(value);
  }
}

//==============================================================================
template<typename Key, typename Data, typename HashFnc>
void SimpleHashTable<Key, Data, HashFnc>::clear()
{
  table_.clear();
  table_.resize(table_size_);
}

} // namespace detail
} // namespace fcl

#endif
