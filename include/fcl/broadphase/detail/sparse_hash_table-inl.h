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

#ifndef FCL_BROADPHASE_SPARSEHASHTABLE_INL_H
#define FCL_BROADPHASE_SPARSEHASHTABLE_INL_H

#include "fcl/broadphase/detail/sparse_hash_table.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
SparseHashTable<Key, Data, HashFnc, TableT>::SparseHashTable(const HashFnc& h)
  : h_(h)
{
  // Do nothing
}

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
void SparseHashTable<Key, Data, HashFnc, TableT>::init(size_t)
{
  table_.clear();
}

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
void SparseHashTable<Key, Data, HashFnc, TableT>::insert(Key key, Data value)
{
  std::vector<unsigned int> indices = h_(key);
  for(size_t i = 0; i < indices.size(); ++i)
    table_[indices[i]].push_back(value);
}

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
std::vector<Data> SparseHashTable<Key, Data, HashFnc, TableT>::query(Key key) const
{
  std::vector<unsigned int> indices = h_(key);
  std::set<Data> result;
  for(size_t i = 0; i < indices.size(); ++i)
  {
    unsigned int index = indices[i];
    typename Table::const_iterator p = table_.find(index);
    if(p != table_.end())
    {
      std::copy((*p).second.begin(), (*p).second.end(), std
                ::inserter(result, result.end()));
    }
  }

  return std::vector<Data>(result.begin(), result.end());
}

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
void SparseHashTable<Key, Data, HashFnc, TableT>::remove(Key key, Data value)
{
  std::vector<unsigned int> indices = h_(key);
  for(size_t i = 0; i < indices.size(); ++i)
  {
    unsigned int index = indices[i];
    table_[index].remove(value);
  }
}

//==============================================================================
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT>
void SparseHashTable<Key, Data, HashFnc, TableT>::clear()
{
  table_.clear();
}

} // namespace detail
} // namespace fcl

#endif
