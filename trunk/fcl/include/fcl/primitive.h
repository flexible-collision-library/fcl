/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/** \author Jia Pan */

#ifndef COLLISION_CHECKING_PRIMITIVE_H
#define COLLISION_CHECKING_PRIMITIVE_H

#include "fcl/BVH_internal.h"
#include "fcl/vec_3f.h"

/** \brief Main namespace */
namespace fcl
{

/** \brief Uncertainty information */
struct Uncertainty
{
  Uncertainty() {}

  Uncertainty(Vec3f Sigma_[3])
  {
    for(int i = 0; i < 3; ++i)
      Sigma[i] = Sigma_[i];
    preprocess();
  }

  /** preprocess performs the eigen decomposition on the Sigma matrix */
  void preprocess()
  {
    matEigen(Sigma, sigma, axis);
  }

  /** sqrt performs the sqrt of Sigma matrix based on the eigen decomposition result, this is useful when the uncertainty matrix is initialized
   * as a square variation matrix
   */
  void sqrt()
  {
    for(int i = 0; i < 3; ++i)
    {
      if(sigma[i] < 0) sigma[i] = 0;
      sigma[i] = std::sqrt(sigma[i]);
    }


    for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        Sigma[i][j] = 0;
      }
    }

    for(int i = 0; i < 3; ++i)
    {
      for(int j = 0; j < 3; ++j)
      {
        Sigma[i][j] += sigma[0] * axis[0][i] * axis[0][j];
        Sigma[i][j] += sigma[1] * axis[1][i] * axis[1][j];
        Sigma[i][j] += sigma[2] * axis[2][i] * axis[2][j];
      }
    }
  }

  /** \brief Variation matrix for uncertainty */
  Vec3f Sigma[3];

  /** \brief Variations along the eigen axes */
  BVH_REAL sigma[3];

  /** \brief eigen axes of uncertainty matrix */
  Vec3f axis[3];
};

/** \brief Simple triangle with 3 indices for points */
struct Triangle
{
  unsigned int vids[3];

  Triangle() {}

  Triangle(unsigned int p1, unsigned int p2, unsigned int p3)
  {
    set(p1, p2, p3);
  }

  inline void set(unsigned int p1, unsigned int p2, unsigned int p3)
  {
    vids[0] = p1; vids[1] = p2; vids[2] = p3;
  }

  inline unsigned int operator[](int i) const { return vids[i]; }

  inline unsigned int& operator[](int i) { return vids[i]; }
};


/** \brief Simple edge with two indices for its endpoints */
struct Edge
{
  unsigned int vids[2];
  unsigned int fids[2];

  Edge()
  {
    vids[0] = -1; vids[1] = -1;
    fids[0] = -1; fids[1] = -1;
  }

  Edge(unsigned int vid0, unsigned int vid1, unsigned int fid)
  {
    vids[0] = vid0;
    vids[1] = vid1;
    fids[0] = fid;
  }

  /** \brief Whether two edges are the same, assuming belongs to the same object */
  bool operator == (const Edge& other) const
  {
    return (vids[0] == other.vids[0]) && (vids[1] == other.vids[1]);
  }

  bool operator < (const Edge& other) const
  {
    if(vids[0] == other.vids[0])
      return vids[1] < other.vids[1];

    return vids[0] < other.vids[0];
  }

};

}


#endif
