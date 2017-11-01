/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Georgia Tech Research Corporation
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
 *   * Neither the name of Georgia Tech Research Corporation nor the names of its
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

/** \author Andrew Price <arprice@gatech.edu> */

#ifndef TEST_FCL_HUNGARIAN_H
#define TEST_FCL_HUNGARIAN_H

#include <Eigen/Core>
#include <vector>

namespace fcl
{

namespace test
{

/**
 * @brief The Hungarian class solves the optimal nxn assignment problem in O(n^3) time.
 * The results are reported as a mapping from x to y (row to column of cost matrix)
 *
 * Formally, the system is defined as follows:
 * Given two sets \f$ X, Y \in \mathbb{R}^n \f$ and a cost matrix \f$ C:X \times Y \rightarrow \mathbb{R} \f$,
 * find a bijection \f$ f:X \rightarrow Y \f$ that minimizes the cost function \f$ \sum_{x\in X}C[x,f(x)] \f$.
 */
template <typename Scalar>
class Hungarian
{
public:
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> CostMatrix;

  Hungarian(const CostMatrix& cost);

  /**
   * @brief getSolutionCost returns the current linear cost of the assignment
   * @return Solution cost of the form C(x1,xToY(x1)) + C(x2,xToY(x2)) + ...
   */
  Scalar getSolutionCost();

  /**
   * @brief getAssignment Returns the optimal choice of y for each x in sequence
   * @return y-index of optimal pairing for each x
   */
  const std::vector<int>& getAssignment() { return xy; }

protected:
  CostMatrix C;                 ///< Cost matrix (nxn)
  const int N;                  ///< Number of pairings to find
  int maxMatch;                 ///< Number of pairings locked so far
  std::vector<Scalar> lx;
  std::vector<Scalar> ly;
  std::vector<int> xy;          ///< Mapping from x to y index
  std::vector<int> yx;          ///< Mapping from y to x index
  std::vector<Scalar> slack;
  std::vector<Scalar> slackx;
  std::vector<int> prev;        ///< Alternating tree
  std::vector<bool> S;          ///< Set of matched x indices
  std::vector<bool> T;          ///< Set of matched y indices

  void augment();
  void relabel();
  void addToTree(int x, int prevx);

};

} // namespace test

} // namespace fcl

#include "test_fcl_hungarian-inl.h"

#endif // TEST_FCL_HUNGARIAN_H
