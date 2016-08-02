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

/** \author Jia Pan */


#ifndef FCL_SHAPE_CONSTRUCT_BOX_H
#define FCL_SHAPE_CONSTRUCT_BOX_H

#include <vector>

#include "fcl/BV/BV.h"
#include "fcl/shape/box.h"

namespace fcl
{

/// @brief construct a box shape (with a configuration) from a given bounding volume
void constructBox(const AABBd& bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBd& bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBRSSd& bv, Boxd& box, Transform3d& tf);

void constructBox(const kIOSd& bv, Boxd& box, Transform3d& tf);

void constructBox(const RSSd& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<16>& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<18>& bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<24>& bv, Boxd& box, Transform3d& tf);

void constructBox(const AABBd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const OBBRSSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const kIOSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const RSSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<16>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<18>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);

void constructBox(const KDOPd<24>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf);


inline void constructBox(const AABBd& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.max_ - bv.min_);
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

inline void constructBox(const OBBd& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.extent * 2);
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

inline void constructBox(const OBBRSSd& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

inline void constructBox(const kIOSd& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
}

inline void constructBox(const RSSd& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf.linear() = bv.axis;
  tf.translation() = bv.Tr;
}

inline void constructBox(const KDOPd<16>& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

inline void constructBox(const KDOPd<18>& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}

inline void constructBox(const KDOPd<24>& bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf.linear().setIdentity();
  tf.translation() = bv.center();
}



inline void constructBox(const AABBd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.max_ - bv.min_);
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

inline void constructBox(const OBBd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.extent * 2);
  tf.linear() = bv.axis;
  tf.translation() = bv.To;
}

inline void constructBox(const OBBRSSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
  tf = tf_bv * tf;
}

inline void constructBox(const kIOSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.obb.extent * 2);
  tf.linear() = bv.obb.axis;
  tf.translation() = bv.obb.To;
  tf = tf_bv * tf;
}

inline void constructBox(const RSSd& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf.linear() = bv.axis;
  tf.translation() = bv.Tr;
  tf = tf_bv * tf;
}

inline void constructBox(const KDOPd<16>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

inline void constructBox(const KDOPd<18>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

inline void constructBox(const KDOPd<24>& bv, const Transform3d& tf_bv, Boxd& box, Transform3d& tf)
{
  box = Boxd(bv.width(), bv.height(), bv.depth());
  tf = tf_bv * Eigen::Translation3d(bv.center());
}

} // namespace fcl

#endif
