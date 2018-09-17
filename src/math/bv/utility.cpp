/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary formdouble, with or without
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
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIEdouble, INCLUDING, BUT NOT
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

#include "fcl/math/bv/utility-inl.h"

namespace fcl {
namespace detail {

//==============================================================================
namespace OBB_fit_functions {
//==============================================================================

//==============================================================================
template
void fit1(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
template
void fit2(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
template
void fit3(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
template
void fit6(const Vector3d* const ps, OBB<double>& bv);

//==============================================================================
template
void fitn(const Vector3d* const ps, int n, OBB<double>& bv);

//==============================================================================
} // namespace OBB_fit_functions
//==============================================================================

//==============================================================================
namespace RSS_fit_functions {
//==============================================================================

//==============================================================================
template
void fit1(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
template
void fit2(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
template
void fit3(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
template
void fit6(const Vector3d* const ps, RSS<double>& bv);

//==============================================================================
template
void fitn(const Vector3d* const ps, int n, RSS<double>& bv);

//==============================================================================
} // namespace RSS_fit_functions
//==============================================================================

//==============================================================================
namespace kIOS_fit_functions {
//==============================================================================

//==============================================================================
template
void fit1(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
template
void fit2(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
template
void fit3(const Vector3d* const ps, kIOS<double>& bv);

//==============================================================================
template
void fitn(const Vector3d* const ps, int n, kIOS<double>& bv);

//==============================================================================
} // namespace kIOS_fit_functions
//==============================================================================

//==============================================================================
namespace OBBRSS_fit_functions {
//==============================================================================

//==============================================================================
template
void fit1(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
template
void fit2(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
template
void fit3(const Vector3d* const ps, OBBRSS<double>& bv);

//==============================================================================
template
void fitn(const Vector3d* const ps, int n, OBBRSS<double>& bv);

//==============================================================================
} // namespace OBBRSS_fit_functions
//==============================================================================

//==============================================================================
template
struct Fitter<double, OBB<double>>;

//==============================================================================
template
struct Fitter<double, RSS<double>>;

//==============================================================================
template
struct Fitter<double, kIOS<double>>;

//==============================================================================
template
struct Fitter<double, OBBRSS<double>>;

//==============================================================================
template
class ConvertBVImpl<double, AABB<double>, AABB<double>>;

//==============================================================================
template
class ConvertBVImpl<double, AABB<double>, OBB<double>>;

//==============================================================================
template
class ConvertBVImpl<double, OBB<double>, OBB<double>>;

//==============================================================================
template
class ConvertBVImpl<double, OBBRSS<double>, OBB<double>>;

//==============================================================================
template
class ConvertBVImpl<double, RSS<double>, OBB<double>>;

//==============================================================================
template
class ConvertBVImpl<double, OBB<double>, RSS<double>>;

//==============================================================================
template
class ConvertBVImpl<double, RSS<double>, RSS<double>>;

//==============================================================================
template
class ConvertBVImpl<double, OBBRSS<double>, RSS<double>>;

//==============================================================================
template
class ConvertBVImpl<double, AABB<double>, RSS<double>>;

} // namespace detail
} // namespace fcl
