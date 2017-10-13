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

#ifndef FCL_BROADPHASE_SPARSEHASHTABLE_H
#define FCL_BROADPHASE_SPARSEHASHTABLE_H

#include <stdexcept>
#include <set>
#include <vector>
#include <list>
#include <unordered_map>

namespace fcl
{

namespace detail
{

template<typename U, typename V>
class FCL_EXPORT unordered_map_hash_table : public std::unordered_map<U, V> {};

/// @brief A hash table implemented using unordered_map
template <typename Key, typename Data, typename HashFnc,
          template<typename, typename> class TableT = unordered_map_hash_table>
class FCL_EXPORT SparseHashTable
{
protected:
  HashFnc h_;
  typedef std::list<Data> Bin;
  typedef TableT<size_t, Bin> Table;
  
  Table table_;
public:
  SparseHashTable(const HashFnc& h);

  /// @brief Init the hash table. The bucket size is dynamically decided.
  void init(size_t);
  
  /// @brief insert one key-value pair into the hash table
  void insert(Key key, Data value);

  /// @brief find the elements whose key is the same as the query
  std::vector<Data> query(Key key) const;

  /// @brief remove one key-value pair from the hash table
  void remove(Key key, Data value);

  /// @brief clear the hash table
  void clear();
};

} // namespace detail
} // namespace fcl

#include "fcl/broadphase/detail/sparse_hash_table-inl.h"

#endif
