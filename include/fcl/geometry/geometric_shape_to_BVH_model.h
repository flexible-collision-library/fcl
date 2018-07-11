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

#ifndef FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_H
#define FCL_SHAPE_GEOMETRICSHAPETOBVHMODEL_H

#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/capsule.h"
#include "fcl/geometry/shape/cone.h"
#include "fcl/geometry/shape/convex.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/ellipsoid.h"
#include "fcl/geometry/shape/halfspace.h"
#include "fcl/geometry/shape/plane.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/triangle_p.h"

namespace fcl
{
/**
@brief    enum class used to indicate whether we simply want to add more primitives to the model
          or finalize it.
*/
enum class FinalizeModel{
	DO,
	DONT
};


/**
@defgroup generateBVHModel
@brief       Create a BVHModel using geometric primitives
@details     The functions in this group can be used to add geometric primitives (Box, Sphere, Ellipsoid, Cylinder, Cone)
             to a BVHModel. It can either close off the model or leave it unfinalized in order to add more primitives later.
@note        All functions in this group have a common sub-set of parameters (listed below). In addition, each has unique
             parameters related to the geometry type being added and how it should be tessellated. These additional parameters
             are documented with their corresponding function
@warning     If this function is used to create a BVHModel containing multiple geometric primitives, the BVHModel inherently 
             represents the *union* of those primitives. The BVHModel structure does not retain any notion of the original 
             geometric primitive.
@param[out]  model The BVHModel to be generated or added to
@param[in]   shape The geometric object to be added to the BVHModel
@param[in]   pose The pose of the geometric object
@param[in]   finalize_model an enum indicating whether the model is final or more submodels can be added later
@return      Return code (as defined by BVHReturnCode) indicating the success of the operation
@{
*/

/// @brief Generate BVH model from box
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Box<typename BV::S>& shape, const Transform3<typename BV::S>& pose, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief Generate BVH model from sphere
@param[in]   seg The number of segments along longitude
@param[in]   ring The number of rings along latitude
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief     Generate BVH model from sphere
@details   The difference between generateBVHModel is that it gives the number of triangles faces N for a sphere with unit radius. For sphere of radius r,
           then the number of triangles is r * r * N so that the area represented by a single triangle is approximately the same.
@param[in] n_faces_for_unit_sphere The number of triangles for a unit-sized sphere
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Sphere<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_sphere, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief      Generate BVH model from ellipsoid
@param[in]  seg The number of segments along longitude
@param[in]  ring The number of rings along latitude
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int seg, unsigned int ring, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief     Generate BVH model from ellipsoid
@details   The difference between generateBVHModel is that it gives the number of triangles faces N for an ellipsoid with unit radii (1, 1, 1). For ellipsoid of radii (a, b, c),
           then the number of triangles is ((a^p * b^p + b^p * c^p + c^p * a^p)/3)^(1/p) * N, where p is 1.6075, so that the area represented by a single triangle is approximately the same.
           Reference: https://en.wikipedia.org/wiki/Ellipsoid<S>#Approximate_formula
@param[in] n_faces_for_unit_ellipsoid The number of faces a unit ellipsoid would have
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Ellipsoid<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int n_faces_for_unit_ellipsoid, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief     Generate BVH model from cylinder, given the number of segments along circle and the number of segments along axis.
@param[in] circle_split_tot The number of segments the bottom plate of the cylinder is split into
@param[in] h_num The number of segments along the axis the cylinder is split into
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot, unsigned int h_num, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief     Generate BVH model from cylinder
@details   Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cylinder with
           larger radius, the number of circle split number is r * tot.
@param[in] circle_split_tot_for_unit_cylinder The number of segments the bottom plate of an equivalent unit-sized cylinder would be split into
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cylinder<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot_for_unit_cylinder, FinalizeModel finalize_model = FinalizeModel::DO);


/**
@brief     Generate BVH model from cone
@param[in] circle_split_tot The number of segments the bottom plate is split into
@param[in] h_num an The number of segments along the axis the cone is split into
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot, unsigned int h_num, FinalizeModel finalize_model = FinalizeModel::DO);

/**
@brief     Generate BVH model from cone
@details   Difference from generateBVHModel: is that it gives the circle split number tot for a cylinder with unit radius. For cone with
           larger radius, the number of circle split number is r * tot.
@param[in] circle_split_tot_for_unit_cone The number of segments the bottom plate of an equivalent unit-sized cone would be split into
**/
template<typename BV>
int generateBVHModel(BVHModel<BV>& model, const Cone<typename BV::S>& shape, const Transform3<typename BV::S>& pose, unsigned int circle_split_tot_for_unit_cone, FinalizeModel finalize_model = FinalizeModel::DO);

/**@} */ // end of doxygen group generateBVHModel

} // namespace fcl

#include "fcl/geometry/geometric_shape_to_BVH_model-inl.h"

#endif