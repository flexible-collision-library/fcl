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

#ifndef FCL_BVH_INTERNAL_H
#define FCL_BVH_INTERNAL_H

namespace fcl
{

typedef double BVH_REAL;

/** \brief States for BVH construction
 * empty->begun->processed ->replace_begun->processed -> ......
 *                         |
 *                         |-> update_begun -> updated -> .....
 * */
enum BVHBuildState
{
  BVH_BUILD_STATE_EMPTY,            // empty state, immediately after constructor
  BVH_BUILD_STATE_BEGUN,            // after beginModel(), state for adding geometry primitives
  BVH_BUILD_STATE_PROCESSED,        // after tree has been build, ready for cd use
  BVH_BUILD_STATE_UPDATE_BEGUN,     // after beginUpdateModel(), state for updating geometry primitives
  BVH_BUILD_STATE_UPDATED,          // after tree has been build for updated geometry, ready for ccd use
  BVH_BUILD_STATE_REPLACE_BEGUN,    // after beginReplaceModel(), state for replacing geometry primitives
};

/** \brief Error code for BVH */
enum BVHReturnCode
{
  BVH_OK = 0,
  BVH_ERR_MODEL_OUT_OF_MEMORY = -1,
  BVH_ERR_OUT_OF_MEMORY = -2,
  BVH_ERR_UNPROCESSED_MODEL = -3,
  BVH_ERR_BUILD_OUT_OF_SEQUENCE = -4,
  BVH_ERR_BUILD_EMPTY_MODEL = -5,
  BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME = -6,
  BVH_ERR_UNSUPPORTED_FUNCTION = -7,
  BVH_ERR_UNUPDATED_MODEL = -8,
  BVH_ERR_INCORRECT_DATA = -9,
  BVH_ERR_UNKNOWN = -10
};

/** \brief BVH model type */
enum BVHModelType
{
  BVH_MODEL_UNKNOWN,
  BVH_MODEL_TRIANGLES,
  BVH_MODEL_POINTCLOUD
};


}

#endif
