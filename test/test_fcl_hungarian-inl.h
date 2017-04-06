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

#include "test_fcl_hungarian.h"

// Implementation of the Hungarian Algorithm derived from
// https://www.topcoder.com/community/data-science/data-science-tutorials/assignment-problem-and-hungarian-algorithm/

namespace fcl
{

namespace test
{

template <typename Scalar>
Hungarian<Scalar>::Hungarian(const Hungarian<Scalar>::CostMatrix& cost)
  : C(-cost), // Negate cost, as this algorithm finds the maximum assignment
    N(C.cols()),
    maxMatch(0),
    lx(N, 0),
    ly(N, 0),
    xy(N,-1),
    yx(N,-1),
    slack(N),
    slackx(N),
    prev(N),
    S(N),
    T(N)
{
  // Initial labelling
  for (int x = 0; x < N; ++x)
    for (int y = 0; y < N; ++y)
      lx[x] = std::max(lx[x], C(x,y));

  // Solve
  augment();
}

template <typename Scalar>
Scalar Hungarian<Scalar>::getSolutionCost()
{
  Scalar cost = 0;
  for (int x = 0; x < N; ++x)
  {
    cost += C(x, xy[x]);
  }
  return -cost;
}

template <typename Scalar>
void Hungarian<Scalar>::relabel()
{
  Scalar delta = std::numeric_limits<Scalar>::infinity();

  for (int y = 0; y < N; ++y) // compute delta with slack
    if (!T[y]) { delta = std::min(delta, slack[y]); }
  assert(std::isfinite(delta));
  for (int x = 0; x < N; ++x) // update X labels
    if (S[x]) { lx[x] -= delta; }
  for (int y = 0; y < N; ++y) // update Y labels
    if (T[y]) { ly[y] += delta; }
  for (int y = 0; y < N; ++y) // update slack array
    if (!T[y]) { slack[y] -= delta; }
}

template <typename Scalar>
void Hungarian<Scalar>::addToTree(int x, int prevx)
{
  S[x] = true;
  prev[x] = prevx;
  for (int y = 0; y < N; ++y)
  {
    Scalar newCost = lx[x] + ly[y] - C(x,y);
    if (newCost < slack[y])
    {
      slack[y] = newCost;
      slackx[y] = x;
    }
  }
}

template <typename Scalar>
void Hungarian<Scalar>::augment()
{
  if (maxMatch == N) { return; }
  int x = -1, y = -1, root = -1;
  std::vector<int> q(N);
  int wr = 0, rd = 0;

  // Initialize sets and tree
  std::fill(S.begin(), S.end(), false);
  std::fill(T.begin(), T.end(), false);
  std::fill(prev.begin(), prev.end(), -1);

  // Find root of tree
  for (x = 0; x < N; ++x)
  {
    if (-1 == xy[x])
    {
      q[wr++] = root = x;
      prev[x] = -2;
      S[x] = true;
      break;
    }
  }

  // Initialize slack array
  for (y = 0; y < N; ++y)
  {
    slack[y] = lx[root] + ly[y] - C(root,y);
    slackx[y] = root;
  }

  while (true)
  {
    while (rd < wr) // build tree with bfs
    {
      x = q[rd++];
      for (y = 0; y < N; ++y)
      {
        if (C(x,y) == lx[x] + ly[y] && !T[y])
        {
          if (-1 == yx[y]) { break; } // found an isolated Y vertex
          T[y] = true; // else y is in T
          q[wr++] = yx[y]; // add target of y to frontier of bfs
          addToTree(yx[y], x);
        }
      }
      if (y < N) { break; } // Breaking out from inmost loop
    }
    if (y < N) { break; } // Breaking out from inmost loop

    relabel();
    wr = rd = 0;

    for (y = 0; y < N; ++y)
    {
      if (!T[y] && 0 == slack[y])
      {
        if (-1 == yx[y])
        {
          x = slackx[y];
          break;
        }
        else
        {
          T[y] = true;
          if (!S[yx[y]])
          {
            q[wr++] = yx[y];
            addToTree(yx[y], slackx[y]);
          }
        }
      }
    }
    if (y < N) { break; }
  }

  if (y < N)
  {
    maxMatch++;
    for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
    {
      ty = xy[cx];
      yx[cy] = cx;
      xy[cx] = cy;
    }
    augment();
  }
}

} // namespace test

} // namespace fcl
