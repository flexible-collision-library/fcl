/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, CNRS-LAAS and AIST
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
 *   * Neither the name of CNRS-LAAS and AIST nor the names of its
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

/** @author Florent Lamiraux */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

template <typename S>
void test_convex_face_normal(fcl::GJKSolverType solver_type, S solver_tolerance, S test_tolerance)
{
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;
  std::vector<fcl::Vector3<S>, Eigen::aligned_allocator<fcl::Vector3<S>>> vertices1, vertices2;
  vertices1.reserve(194);

  vertices1.push_back(fcl::Vector3<S>(-0.051848,0.004004,0.081271));
  vertices1.push_back(fcl::Vector3<S>(-0.050529,0.008905,0.087501));
  vertices1.push_back(fcl::Vector3<S>(-0.048489,0.019294,0.081244));
  vertices1.push_back(fcl::Vector3<S>(-0.024230,0.006800,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.024514,-0.005704,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.051320,-0.008988,0.081373));
  vertices1.push_back(fcl::Vector3<S>(-0.051280,-0.004545,0.087496));
  vertices1.push_back(fcl::Vector3<S>(-0.051270,0.000009,0.087603));
  vertices1.push_back(fcl::Vector3<S>(-0.049250,0.000528,0.099913));
  vertices1.push_back(fcl::Vector3<S>(-0.048519,0.007491,0.100499));
  vertices1.push_back(fcl::Vector3<S>(-0.047179,0.007502,0.108347));
  vertices1.push_back(fcl::Vector3<S>(-0.046184,0.015391,0.103461));
  vertices1.push_back(fcl::Vector3<S>(-0.048214,0.017536,0.087539));
  vertices1.push_back(fcl::Vector3<S>(-0.020220,0.014689,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.046560,0.021927,0.087608));
  vertices1.push_back(fcl::Vector3<S>(-0.041840,0.030070,0.087479));
  vertices1.push_back(fcl::Vector3<S>(-0.040531,0.032622,0.081254));
  vertices1.push_back(fcl::Vector3<S>(-0.018728,-0.016806,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.048600,-0.018392,0.081254));
  vertices1.push_back(fcl::Vector3<S>(-0.050488,-0.008896,0.087603));
  vertices1.push_back(fcl::Vector3<S>(-0.048529,-0.007493,0.100499));
  vertices1.push_back(fcl::Vector3<S>(-0.046996,-0.007951,0.108767));
  vertices1.push_back(fcl::Vector3<S>(-0.043007,0.007481,0.115212));
  vertices1.push_back(fcl::Vector3<S>(-0.045251,0.014140,0.110046));
  vertices1.push_back(fcl::Vector3<S>(-0.042987,0.001199,0.115889));
  vertices1.push_back(fcl::Vector3<S>(-0.043170,0.023422,0.100564));
  vertices1.push_back(fcl::Vector3<S>(-0.013004,0.021551,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.039617,0.028993,0.100494));
  vertices1.push_back(fcl::Vector3<S>(-0.034674,0.034696,0.100650));
  vertices1.push_back(fcl::Vector3<S>(-0.036369,0.036384,0.087608));
  vertices1.push_back(fcl::Vector3<S>(-0.040460,0.024489,0.110138));
  vertices1.push_back(fcl::Vector3<S>(-0.032847,0.039403,0.087528));
  vertices1.push_back(fcl::Vector3<S>(-0.024392,0.046011,0.081158));
  vertices1.push_back(fcl::Vector3<S>(-0.008314,-0.023759,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.032066,-0.041072,0.081104));
  vertices1.push_back(fcl::Vector3<S>(-0.039841,-0.033417,0.081367));
  vertices1.push_back(fcl::Vector3<S>(-0.046469,-0.022153,0.087485));
  vertices1.push_back(fcl::Vector3<S>(-0.046540,-0.021889,0.087657));
  vertices1.push_back(fcl::Vector3<S>(-0.045738,-0.014142,0.108385));
  vertices1.push_back(fcl::Vector3<S>(-0.043129,-0.007483,0.115282));
  vertices1.push_back(fcl::Vector3<S>(-0.030614,0.006993,0.125974));
  vertices1.push_back(fcl::Vector3<S>(-0.040622,0.015147,0.115734));
  vertices1.push_back(fcl::Vector3<S>(-0.031264,-0.002370,0.125969));
  vertices1.push_back(fcl::Vector3<S>(-0.001006,0.025150,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.010700,0.051023,0.081002));
  vertices1.push_back(fcl::Vector3<S>(-0.027843,0.038590,0.109224));
  vertices1.push_back(fcl::Vector3<S>(-0.039272,0.018227,0.115599));
  vertices1.push_back(fcl::Vector3<S>(-0.035720,0.025120,0.115314));
  vertices1.push_back(fcl::Vector3<S>(-0.030980,0.029410,0.116707));
  vertices1.push_back(fcl::Vector3<S>(-0.028990,0.039596,0.100499));
  vertices1.push_back(fcl::Vector3<S>(-0.025651,0.044435,0.087501));
  vertices1.push_back(fcl::Vector3<S>(-0.017531,0.048197,0.087549));
  vertices1.push_back(fcl::Vector3<S>(0.004150,-0.024827,0.071105));
  vertices1.push_back(fcl::Vector3<S>(-0.003706,-0.051909,0.081238));
  vertices1.push_back(fcl::Vector3<S>(-0.019094,-0.048483,0.081163));
  vertices1.push_back(fcl::Vector3<S>(-0.025641,-0.044396,0.087565));
  vertices1.push_back(fcl::Vector3<S>(-0.029487,-0.042190,0.087496));
  vertices1.push_back(fcl::Vector3<S>(-0.032949,-0.039273,0.087603));
  vertices1.push_back(fcl::Vector3<S>(-0.039303,-0.032980,0.087501));
  vertices1.push_back(fcl::Vector3<S>(-0.043150,-0.023403,0.100542));
  vertices1.push_back(fcl::Vector3<S>(-0.045017,-0.018717,0.102929));
  vertices1.push_back(fcl::Vector3<S>(-0.039617,-0.029005,0.100499));
  vertices1.push_back(fcl::Vector3<S>(-0.038531,-0.027694,0.109605));
  vertices1.push_back(fcl::Vector3<S>(-0.040866,-0.020648,0.113293));
  vertices1.push_back(fcl::Vector3<S>(-0.040145,-0.015454,0.115954));
  vertices1.push_back(fcl::Vector3<S>(-0.028280,-0.013695,0.125974));
  vertices1.push_back(fcl::Vector3<S>(-0.027579,0.002683,0.125985));
  vertices1.push_back(fcl::Vector3<S>(-0.018109,0.014791,0.126001));
  vertices1.push_back(fcl::Vector3<S>(-0.018444,0.020321,0.125990));
  vertices1.push_back(fcl::Vector3<S>(-0.026432,0.017342,0.125877));
  vertices1.push_back(fcl::Vector3<S>(0.011225,0.022527,0.071105));
  vertices1.push_back(fcl::Vector3<S>(0.003632,0.051928,0.081115));
  vertices1.push_back(fcl::Vector3<S>(-0.008893,0.050484,0.087603));
  vertices1.push_back(fcl::Vector3<S>(-0.003757,0.051399,0.087512));
  vertices1.push_back(fcl::Vector3<S>(-0.024960,0.035489,0.115535));
  vertices1.push_back(fcl::Vector3<S>(-0.020342,0.040653,0.113820));
  vertices1.push_back(fcl::Vector3<S>(-0.014252,0.044150,0.112755));
  vertices1.push_back(fcl::Vector3<S>(-0.023408,0.043144,0.100574));
  vertices1.push_back(fcl::Vector3<S>(-0.016840,0.026705,0.125867));
  vertices1.push_back(fcl::Vector3<S>(-0.014354,0.046875,0.100741));
  vertices1.push_back(fcl::Vector3<S>(0.015599,-0.019754,0.071105));
  vertices1.push_back(fcl::Vector3<S>(0.028622,-0.043807,0.081201));
  vertices1.push_back(fcl::Vector3<S>(0.010413,-0.051076,0.081002));
  vertices1.push_back(fcl::Vector3<S>(-0.000001,-0.051309,0.087501));
  vertices1.push_back(fcl::Vector3<S>(-0.008893,-0.050496,0.087555));
  vertices1.push_back(fcl::Vector3<S>(-0.013420,-0.049693,0.087506));
  vertices1.push_back(fcl::Vector3<S>(-0.017531,-0.048178,0.087603));
  vertices1.push_back(fcl::Vector3<S>(-0.023296,-0.043227,0.100757));
  vertices1.push_back(fcl::Vector3<S>(-0.027924,-0.038693,0.108573));
  vertices1.push_back(fcl::Vector3<S>(-0.034583,-0.034779,0.100687));
  vertices1.push_back(fcl::Vector3<S>(-0.030675,-0.030154,0.115857));
  vertices1.push_back(fcl::Vector3<S>(-0.035730,-0.025132,0.115298));
  vertices1.push_back(fcl::Vector3<S>(-0.036999,-0.016948,0.118072));
  vertices1.push_back(fcl::Vector3<S>(-0.019652,-0.025010,0.125963));
  vertices1.push_back(fcl::Vector3<S>(-0.015846,-0.019561,0.126001));
  vertices1.push_back(fcl::Vector3<S>(-0.022514,0.002002,0.125996));
  vertices1.push_back(fcl::Vector3<S>(0.016462,0.019904,0.126012));
  vertices1.push_back(fcl::Vector3<S>(-0.002295,0.026705,0.125996));
  vertices1.push_back(fcl::Vector3<S>(-0.006599,0.030680,0.125979));
  vertices1.push_back(fcl::Vector3<S>(0.020695,0.014323,0.071105));
  vertices1.push_back(fcl::Vector3<S>(0.022562,0.047251,0.081195));
  vertices1.push_back(fcl::Vector3<S>(-0.000001,0.051267,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.008900,0.050484,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.013417,0.049691,0.087490));
  vertices1.push_back(fcl::Vector3<S>(-0.003016,0.047780,0.108460));
  vertices1.push_back(fcl::Vector3<S>(-0.015298,0.040348,0.115707));
  vertices1.push_back(fcl::Vector3<S>(0.002739,0.040674,0.117889));
  vertices1.push_back(fcl::Vector3<S>(0.007205,0.046509,0.111578));
  vertices1.push_back(fcl::Vector3<S>(0.023192,-0.009791,0.071105));
  vertices1.push_back(fcl::Vector3<S>(0.039899,-0.033478,0.081254));
  vertices1.push_back(fcl::Vector3<S>(0.029800,-0.041936,0.087614));
  vertices1.push_back(fcl::Vector3<S>(0.025648,-0.044437,0.087501));
  vertices1.push_back(fcl::Vector3<S>(0.017528,-0.048178,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.036488,-0.036264,0.087506));
  vertices1.push_back(fcl::Vector3<S>(0.013326,-0.049724,0.087506));
  vertices1.push_back(fcl::Vector3<S>(0.008900,-0.050486,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.000060,-0.047894,0.108998));
  vertices1.push_back(fcl::Vector3<S>(-0.014161,-0.046521,0.102907));
  vertices1.push_back(fcl::Vector3<S>(-0.021479,-0.041621,0.110960));
  vertices1.push_back(fcl::Vector3<S>(-0.025133,-0.035725,0.115250));
  vertices1.push_back(fcl::Vector3<S>(-0.017307,-0.038998,0.116744));
  vertices1.push_back(fcl::Vector3<S>(-0.000519,-0.032055,0.125630));
  vertices1.push_back(fcl::Vector3<S>(-0.001879,-0.026931,0.125996));
  vertices1.push_back(fcl::Vector3<S>(0.002191,-0.026728,0.126001));
  vertices1.push_back(fcl::Vector3<S>(0.019731,-0.017466,0.126006));
  vertices1.push_back(fcl::Vector3<S>(0.024674,0.020524,0.125974));
  vertices1.push_back(fcl::Vector3<S>(0.013336,0.028423,0.125990));
  vertices1.push_back(fcl::Vector3<S>(0.027617,0.000456,0.126001));
  vertices1.push_back(fcl::Vector3<S>(0.003805,0.031392,0.125797));
  vertices1.push_back(fcl::Vector3<S>(0.025039,0.002581,0.071105));
  vertices1.push_back(fcl::Vector3<S>(0.039432,0.031595,0.081002));
  vertices1.push_back(fcl::Vector3<S>(0.044568,0.027376,0.081168));
  vertices1.push_back(fcl::Vector3<S>(0.032946,0.039261,0.087711));
  vertices1.push_back(fcl::Vector3<S>(0.036529,0.036262,0.087426));
  vertices1.push_back(fcl::Vector3<S>(0.017528,0.048176,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.018249,0.045269,0.102241));
  vertices1.push_back(fcl::Vector3<S>(0.022268,0.042199,0.108439));
  vertices1.push_back(fcl::Vector3<S>(0.025597,0.044466,0.087533));
  vertices1.push_back(fcl::Vector3<S>(0.013965,0.047068,0.100440));
  vertices1.push_back(fcl::Vector3<S>(0.014412,0.043408,0.113159));
  vertices1.push_back(fcl::Vector3<S>(0.051846,-0.004006,0.081287));
  vertices1.push_back(fcl::Vector3<S>(0.049288,-0.016795,0.081271));
  vertices1.push_back(fcl::Vector3<S>(0.039270,-0.032960,0.087565));
  vertices1.push_back(fcl::Vector3<S>(0.044375,-0.025792,0.087313));
  vertices1.push_back(fcl::Vector3<S>(0.029404,-0.039649,0.100494));
  vertices1.push_back(fcl::Vector3<S>(0.023364,-0.043136,0.100467));
  vertices1.push_back(fcl::Vector3<S>(0.019010,-0.044874,0.102988));
  vertices1.push_back(fcl::Vector3<S>(0.014097,-0.046084,0.106740));
  vertices1.push_back(fcl::Vector3<S>(0.038721,-0.027866,0.108606));
  vertices1.push_back(fcl::Vector3<S>(0.001704,-0.045932,0.113121));
  vertices1.push_back(fcl::Vector3<S>(-0.013978,-0.044508,0.111777));
  vertices1.push_back(fcl::Vector3<S>(0.014808,-0.040960,0.115373));
  vertices1.push_back(fcl::Vector3<S>(0.013306,-0.028405,0.125942));
  vertices1.push_back(fcl::Vector3<S>(0.022715,-0.022153,0.125888));
  vertices1.push_back(fcl::Vector3<S>(0.029739,-0.010492,0.125931));
  vertices1.push_back(fcl::Vector3<S>(0.031434,0.003669,0.125850));
  vertices1.push_back(fcl::Vector3<S>(0.039320,0.017515,0.115954));
  vertices1.push_back(fcl::Vector3<S>(0.035179,0.025445,0.115825));
  vertices1.push_back(fcl::Vector3<S>(0.025130,0.035723,0.115293));
  vertices1.push_back(fcl::Vector3<S>(0.016838,0.039088,0.116287));
  vertices1.push_back(fcl::Vector3<S>(0.051318,0.008986,0.081373));
  vertices1.push_back(fcl::Vector3<S>(0.046730,0.021572,0.087533));
  vertices1.push_back(fcl::Vector3<S>(0.039270,0.032958,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.039635,0.029105,0.100494));
  vertices1.push_back(fcl::Vector3<S>(0.044395,0.025628,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.028987,0.039596,0.100499));
  vertices1.push_back(fcl::Vector3<S>(0.027759,0.038549,0.109740));
  vertices1.push_back(fcl::Vector3<S>(0.038417,0.027773,0.110240));
  vertices1.push_back(fcl::Vector3<S>(0.051267,-0.000123,0.087673));
  vertices1.push_back(fcl::Vector3<S>(0.050496,-0.008907,0.087565));
  vertices1.push_back(fcl::Vector3<S>(0.049785,-0.013410,0.087496));
  vertices1.push_back(fcl::Vector3<S>(0.051277,0.004553,0.087533));
  vertices1.push_back(fcl::Vector3<S>(0.048171,-0.017527,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.039594,-0.028995,0.100499));
  vertices1.push_back(fcl::Vector3<S>(0.044801,-0.019123,0.102440));
  vertices1.push_back(fcl::Vector3<S>(0.042112,-0.022255,0.108686));
  vertices1.push_back(fcl::Vector3<S>(0.027455,-0.038632,0.110326));
  vertices1.push_back(fcl::Vector3<S>(0.018858,-0.039730,0.115336));
  vertices1.push_back(fcl::Vector3<S>(0.035413,-0.024989,0.115513));
  vertices1.push_back(fcl::Vector3<S>(0.025110,-0.035704,0.115212));
  vertices1.push_back(fcl::Vector3<S>(0.029363,-0.028161,0.117932));
  vertices1.push_back(fcl::Vector3<S>(0.039533,-0.015535,0.116545));
  vertices1.push_back(fcl::Vector3<S>(0.042731,-0.007321,0.115911));
  vertices1.push_back(fcl::Vector3<S>(0.042802,0.007430,0.115508));
  vertices1.push_back(fcl::Vector3<S>(0.041503,0.014709,0.115045));
  vertices1.push_back(fcl::Vector3<S>(0.041685,0.021480,0.111003));
  vertices1.push_back(fcl::Vector3<S>(0.048181,0.017525,0.087571));
  vertices1.push_back(fcl::Vector3<S>(0.050485,0.008894,0.087603));
  vertices1.push_back(fcl::Vector3<S>(0.044000,0.022161,0.100257));
  vertices1.push_back(fcl::Vector3<S>(0.046030,0.014028,0.107133));
  vertices1.push_back(fcl::Vector3<S>(0.048516,-0.007493,0.100499));
  vertices1.push_back(fcl::Vector3<S>(0.047034,0.007878,0.108638));
  vertices1.push_back(fcl::Vector3<S>(0.045512,-0.013664,0.109713));
  vertices1.push_back(fcl::Vector3<S>(0.048608,0.007471,0.100499));

  std::vector<int> faces1 = {3,0,1,2,3,0,2,3,3,0,3,4,3,0,4,5,3,0,5,6,3,0,6,7,3,0,7,1,3,1,7,8,3,1,8,9,3,1,9,10,
                             3,1,10,11,3,1,11,12,3,1,12,2,3,2,13,3,3,2,12,14,3,2,14,15,3,2,15,16,3,2,16,13,13,
                             3,13,26,43,70,99,129,108,80,52,33,17,4,3,4,17,18,3,4,18,5,3,5,18,19,3,5,19,6,3,6,
                             20,21,3,6,21,8,3,6,8,7,3,6,19,20,3,8,21,10,3,8,10,9,3,10,22,23,3,10,23,11,3,10,21,
                             24,3,10,24,22,3,11,25,14,3,11,14,12,3,11,23,25,3,13,16,26,3,14,25,15,3,15,27,28,3,
                             15,28,29,3,15,29,16,3,15,25,30,3,15,30,27,3,16,29,31,3,16,31,32,3,16,32,26,3,17,33,
                             34,3,17,34,35,3,17,35,18,3,18,36,37,3,18,37,38,3,18,38,19,3,18,35,36,3,19,21,20,3,19,
                             38,21,3,21,38,39,3,21,39,24,3,22,40,41,3,22,41,23,3,22,24,40,3,23,41,30,3,23,30,25,
                             3,24,39,42,3,24,42,40,3,26,32,44,3,26,44,43,3,27,30,28,3,28,45,29,3,28,30,45,3,29,45,31,
                             3,30,41,46,3,30,46,47,3,30,47,48,3,30,48,45,3,31,45,49,3,31,49,50,3,31,50,32,3,32,50,51,
                             3,32,51,44,3,33,52,53,3,33,53,54,3,33,54,34,3,34,54,55,3,34,55,56,3,34,56,57,3,34,57,35,
                             3,35,57,58,3,35,58,36,3,36,59,60,3,36,60,37,3,36,58,61,3,36,61,59,3,37,60,38,3,38,60,62,
                             3,38,62,63,3,38,63,39,3,39,63,64,3,39,64,65,3,39,65,42,3,40,42,66,3,40,66,67,3,40,67,68,
                             3,40,68,69,3,40,69,41,3,41,69,46,3,42,65,66,3,43,44,71,3,43,71,70,3,44,51,72,3,44,72,73,
                             3,44,73,71,3,45,48,74,3,45,74,75,3,45,75,76,3,45,76,77,3,45,77,50,3,45,50,49,3,46,69,47,
                             3,47,69,48,3,48,69,78,3,48,78,74,3,50,77,51,3,51,79,72,3,51,77,79,3,52,80,81,3,52,81,82,
                             3,52,82,53,3,53,83,84,3,53,84,85,3,53,85,54,3,53,82,83,3,54,85,86,3,54,86,55,3,55,87,56,
                             3,55,86,87,3,56,87,88,3,56,88,57,3,57,88,89,3,57,89,58,3,58,89,61,3,59,61,62,3,59,62,60,
                             3,61,89,62,3,62,89,88,3,62,88,90,3,62,90,91,3,62,91,63,3,63,91,92,3,63,92,64,3,64,92,65,
                             3,65,92,91,3,65,91,93,3,65,93,94,3,65,94,66,3,66,95,67,3,66,94,95,3,67,95,94,3,67,94,96,
                             3,67,96,97,3,67,97,68,3,68,97,98,3,68,98,78,3,68,78,69,3,70,71,100,3,70,100,99,3,71,73,101,
                             3,71,101,102,3,71,102,103,3,71,103,100,3,72,79,104,3,72,104,73,3,73,104,101,3,74,78,75,
                             3,75,105,76,3,75,78,105,3,76,105,98,3,76,98,106,3,76,106,107,3,76,107,104,3,76,104,79,
                             3,76,79,77,3,78,98,105,3,80,108,109,3,80,109,81,3,81,110,111,3,81,111,112,3,81,112,82,
                             3,81,109,113,3,81,113,110,3,82,112,114,3,82,114,115,3,82,115,83,3,83,115,116,3,83,116,84,
                             3,84,116,85,3,85,116,117,3,85,117,86,3,86,117,87,3,87,118,88,3,87,117,118,3,88,119,90,
                             3,88,118,119,3,90,119,93,3,90,93,91,3,93,119,120,3,93,120,121,3,93,121,122,3,93,122,94,
                             3,94,122,123,3,94,123,124,3,94,124,96,3,96,125,126,3,96,126,97,3,96,124,127,3,96,127,125,
                             3,97,126,98,3,98,126,128,3,98,128,106,3,99,100,130,3,99,130,131,3,99,131,129,3,100,132,133,
                             3,100,133,131,3,100,131,130,3,100,103,134,3,100,134,135,3,100,135,136,3,100,136,137,
                             3,100,137,132,3,101,104,107,3,101,107,102,3,102,107,103,3,103,107,138,3,103,138,135,
                             3,103,135,134,3,106,139,107,3,106,128,139,3,107,139,136,3,107,136,135,3,107,135,138,
                             3,108,140,141,3,108,141,109,3,108,129,140,3,109,142,113,3,109,141,143,3,109,143,142,
                             3,110,113,144,3,110,144,111,3,111,144,145,3,111,145,146,3,111,146,112,3,112,146,147,
                             3,112,147,114,3,113,142,148,3,113,148,144,3,114,147,115,3,115,147,116,3,116,147,149,
                             3,116,149,150,3,116,150,117,3,117,150,118,3,118,150,120,3,118,120,119,3,120,150,149,
                             3,120,149,121,3,121,149,151,3,121,151,152,3,121,152,122,3,122,152,123,3,123,152,124,
                             3,124,152,153,3,124,153,154,3,124,154,127,3,125,127,155,3,125,155,156,3,125,156,157,
                             3,125,157,158,3,125,158,126,3,126,158,159,3,126,159,128,3,127,154,155,3,128,159,139,
                             3,129,131,160,3,129,160,140,3,131,161,160,3,131,133,162,3,131,162,163,3,131,163,164,
                             3,131,164,161,3,132,165,166,3,132,166,133,3,132,137,165,3,133,166,167,3,133,167,163,
                             3,133,163,162,3,136,139,166,3,136,166,137,3,137,166,165,3,139,159,158,3,139,158,166,
                             3,140,168,169,3,140,169,170,3,140,170,141,3,140,160,171,3,140,171,168,3,141,170,172,
                             3,141,172,143,3,142,143,173,3,142,173,148,3,143,172,174,3,143,174,175,3,143,175,148,
                             3,143,148,173,3,144,148,176,3,144,176,145,3,145,176,146,3,146,176,147,3,147,176,177,
                             3,147,177,151,3,147,151,149,3,148,175,178,3,148,178,176,3,151,177,152,3,152,177,179,
                             3,152,179,153,3,153,178,154,3,153,179,180,3,153,180,178,3,154,181,182,3,154,182,155,
                             3,154,178,181,3,155,183,184,3,155,184,156,3,155,182,183,3,156,185,157,3,156,184,185,
                             3,157,185,167,3,157,167,158,3,158,167,166,3,160,161,186,3,160,186,187,3,160,187,171,
                             3,161,164,188,3,161,188,189,3,161,189,186,3,163,167,188,3,163,188,164,3,167,185,188,
                             3,168,190,169,3,168,171,191,3,168,191,190,3,169,190,170,3,170,190,192,3,170,192,174,
                             3,170,174,172,3,171,187,193,3,171,193,191,3,174,192,175,3,175,192,181,3,175,181,178,
                             3,176,179,177,3,176,178,179,3,178,180,179,3,181,192,182,3,182,192,191,3,182,191,183,
                             3,183,191,184,3,184,191,189,3,184,189,185,3,185,189,188,3,186,189,187,3,187,189,193,
                             3,189,191,193,3,190,191,192};

  CollisionGeometryPtr_t convexGeometry1 (new fcl::Convex<S>(vertices1.size(), vertices1.data(), 374, faces1.data()));

  vertices2.reserve(42);
  vertices2.push_back(fcl::Vector3<S>(0.000000,-0.250000,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.146904,-0.202232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.069076,-0.212657,0.111785));
  vertices2.push_back(fcl::Vector3<S>(-0.065678,-0.202232,0.131412));
  vertices2.push_back(fcl::Vector3<S>(-0.146904,-0.202232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(-0.069076,-0.212657,-0.111785));
  vertices2.push_back(fcl::Vector3<S>(0.065678,-0.202232,-0.131412));
  vertices2.push_back(fcl::Vector3<S>(0.237764,-0.077232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.172039,-0.124951,0.131412));
  vertices2.push_back(fcl::Vector3<S>(0.180883,-0.131412,-0.111785));
  vertices2.push_back(fcl::Vector3<S>(0.040589,-0.124951,0.212657));
  vertices2.push_back(fcl::Vector3<S>(-0.106314,-0.077232,0.212657));
  vertices2.push_back(fcl::Vector3<S>(-0.180883,-0.131412,0.111785));
  vertices2.push_back(fcl::Vector3<S>(-0.237764,-0.077232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(-0.172039,-0.124951,-0.131412));
  vertices2.push_back(fcl::Vector3<S>(-0.040589,-0.124951,-0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.106314,-0.077232,-0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.212629,0.000000,-0.131412));
  vertices2.push_back(fcl::Vector3<S>(0.237764,0.077232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.223567,0.000000,0.111785));
  vertices2.push_back(fcl::Vector3<S>(0.131404,0.000000,0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.000000,0.000000,0.250000));
  vertices2.push_back(fcl::Vector3<S>(-0.106314,0.077232,0.212657));
  vertices2.push_back(fcl::Vector3<S>(-0.212629,0.000000,0.131412));
  vertices2.push_back(fcl::Vector3<S>(-0.223567,0.000000,-0.111785));
  vertices2.push_back(fcl::Vector3<S>(-0.237764,0.077232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(-0.131404,0.000000,-0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.000000,0.000000,-0.250000));
  vertices2.push_back(fcl::Vector3<S>(0.106314,0.077232,-0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.180883,0.131412,-0.111785));
  vertices2.push_back(fcl::Vector3<S>(0.146904,0.202232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.172039,0.124951,0.131412));
  vertices2.push_back(fcl::Vector3<S>(0.040589,0.124951,0.212657));
  vertices2.push_back(fcl::Vector3<S>(-0.065678,0.202232,0.131412));
  vertices2.push_back(fcl::Vector3<S>(-0.180883,0.131412,0.111785));
  vertices2.push_back(fcl::Vector3<S>(-0.172039,0.124951,-0.131412));
  vertices2.push_back(fcl::Vector3<S>(-0.146904,0.202232,0.000000));
  vertices2.push_back(fcl::Vector3<S>(-0.040589,0.124951,-0.212657));
  vertices2.push_back(fcl::Vector3<S>(0.065678,0.202232,-0.131412));
  vertices2.push_back(fcl::Vector3<S>(0.000000,0.250000,0.000000));
  vertices2.push_back(fcl::Vector3<S>(0.069076,0.212657,0.111785));
  vertices2.push_back(fcl::Vector3<S>(-0.069076,0.212657,-0.111785));

  std::vector<int> faces2 = {3,0,1,2,3,0,2,3,3,0,3,4,3,0,4,5,3,0,5,6,3,0,6,1,3,1,7,8,3,1,8,2,
                             3,1,6,9,3,1,9,7,3,2,8,10,3,2,10,3,3,3,10,11,3,3,11,12,3,3,12,4,3,4,12,13,
                             3,4,13,14,3,4,14,5,3,5,14,15,3,5,15,6,3,6,15,16,3,6,16,9,3,7,9,17,3,7,17,18,
                             3,7,18,19,3,7,19,8,3,8,19,20,3,8,20,10,3,9,16,17,3,10,20,21,3,10,21,11,3,11,21,22,
                             3,11,22,23,3,11,23,12,3,12,23,13,3,13,24,14,3,13,23,25,3,13,25,24,3,14,26,15,3,14,24,26,
                             3,15,27,16,3,15,26,27,3,16,28,17,3,16,27,28,3,17,28,29,3,17,29,18,3,18,30,31,3,18,31,19,
                             3,18,29,30,3,19,31,20,3,20,31,32,3,20,32,21,3,21,32,22,3,22,32,33,3,22,33,34,3,22,34,23,
                             3,23,34,25,3,24,25,35,3,24,35,26,3,25,34,36,3,25,36,35,3,26,35,37,3,26,37,27,3,27,37,28,
                             3,28,37,38,3,28,38,29,3,29,38,30,3,30,39,40,3,30,40,31,3,30,38,39,3,31,40,32,3,32,40,33,
                             3,33,40,39,3,33,39,36,3,33,36,34,3,35,41,37,3,35,36,41,3,36,39,41,3,37,41,38,3,38,41,39};

  CollisionGeometryPtr_t convexGeometry2 (new fcl::Convex<S>(vertices2.size(), vertices2.data(), 80, faces2.data()));

  // Enable computation of nearest points
  fcl::DistanceRequest<S> distanceRequest (true, true);
  fcl::DistanceResult<S> distanceResult;

  distanceRequest.gjk_solver_type = solver_type;
  distanceRequest.distance_tolerance = solver_tolerance;

  fcl::Transform3<S> tf1;
  tf1(0,0)=-0.9993982694846256;
  tf1(1,0)=-4.7264240395566515e-15;
  tf1(2,0)=-0.03468571681737781;
  tf1(3,0)=0.0;
  tf1(0,1)=-4.950254069181713e-15;
  tf1(1,1)=1.0;
  tf1(2,1)=6.3672119522733725e-15;
  tf1(3,1)=0.0;
  tf1(0,2)=0.03468571681737781;
  tf1(1,2)=6.535083717361544e-15;
  tf1(2,2)=-0.9993982694846256;
  tf1(3,2)=0.0;
  tf1(0,3)=0.5141949360204755;
  tf1(1,3)=4.413795180657667e-15;
  tf1(2,3)=0.7478853327103305;
  tf1(3,3)=1.0;

  fcl::Transform3<S> tf2(fcl::Translation3<S>(fcl::Vector3<S> (0.5, 0, 0.55)));
  fcl::CollisionObject<S> convex1 (convexGeometry1, tf1);
  fcl::CollisionObject<S> convex2 (convexGeometry2, tf2);

  // test distance
  fcl::distance (&convex1, &convex2, distanceRequest, distanceResult);

}

template <typename S>
void test_box_cylinder(fcl::GJKSolverType solver_type, S solver_tolerance, S test_tolerance)
{
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;

  CollisionGeometryPtr_t geometry1 (new fcl::Box<S>(1, 1, 1));
  CollisionGeometryPtr_t geometry2 (new fcl::Cylinder<S>(0.25, 0.25));

  // Enable computation of nearest points
  fcl::DistanceRequest<S> distanceRequest (true, true);
  fcl::DistanceResult<S> distanceResult;

  distanceRequest.gjk_solver_type = solver_type;
  distanceRequest.distance_tolerance = solver_tolerance;

  fcl::Transform3<S> tf1, tf2;

  //////////////////////////////////////
  // Test when object is in collision
  //////////////////////////////////////
  tf1.setIdentity();
  tf2.setIdentity();
  tf2.translation()(0) = 0.2;

  fcl::CollisionObject<S> collision_object1 (geometry1, tf1);
  fcl::CollisionObject<S> collision_object2 (geometry2, tf2);

  // test distance
  fcl::distance (&collision_object1, &collision_object2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, -0.55, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.5, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], 0.0, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[1][0], -0.05, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[1][1], 0.0, test_tolerance);

  ////////////////////////
  // Test object distance
  ////////////////////////
  distanceResult.clear();
  tf2.setIdentity();
  tf2.translation()(0) = 1.0;

  collision_object1 = fcl::CollisionObject<S>(geometry1, tf1);
  collision_object2 = fcl::CollisionObject<S>(geometry2, tf2);

  // test distance
  fcl::distance (&collision_object1, &collision_object2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, 0.25, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.5, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], 0.0, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.75, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], 0.0, test_tolerance);
}

template <typename S>
void test_convex_sphere_sphere(fcl::GJKSolverType solver_type, S solver_tolerance, S test_tolerance)
{
  std::vector<fcl::Vector3<S>, Eigen::aligned_allocator<fcl::Vector3<S>>> vertices;
  using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<S>>;

  vertices.reserve(42);
  vertices.push_back(fcl::Vector3<S>(0.000000,-0.250000,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.146904,-0.202232,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.069076,-0.212657,0.111785));
  vertices.push_back(fcl::Vector3<S>(-0.065678,-0.202232,0.131412));
  vertices.push_back(fcl::Vector3<S>(-0.146904,-0.202232,0.000000));
  vertices.push_back(fcl::Vector3<S>(-0.069076,-0.212657,-0.111785));
  vertices.push_back(fcl::Vector3<S>(0.065678,-0.202232,-0.131412));
  vertices.push_back(fcl::Vector3<S>(0.237764,-0.077232,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.172039,-0.124951,0.131412));
  vertices.push_back(fcl::Vector3<S>(0.180883,-0.131412,-0.111785));
  vertices.push_back(fcl::Vector3<S>(0.040589,-0.124951,0.212657));
  vertices.push_back(fcl::Vector3<S>(-0.106314,-0.077232,0.212657));
  vertices.push_back(fcl::Vector3<S>(-0.180883,-0.131412,0.111785));
  vertices.push_back(fcl::Vector3<S>(-0.237764,-0.077232,0.000000));
  vertices.push_back(fcl::Vector3<S>(-0.172039,-0.124951,-0.131412));
  vertices.push_back(fcl::Vector3<S>(-0.040589,-0.124951,-0.212657));
  vertices.push_back(fcl::Vector3<S>(0.106314,-0.077232,-0.212657));
  vertices.push_back(fcl::Vector3<S>(0.212629,0.000000,-0.131412));
  vertices.push_back(fcl::Vector3<S>(0.237764,0.077232,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.223567,0.000000,0.111785));
  vertices.push_back(fcl::Vector3<S>(0.131404,0.000000,0.212657));
  vertices.push_back(fcl::Vector3<S>(0.000000,0.000000,0.250000));
  vertices.push_back(fcl::Vector3<S>(-0.106314,0.077232,0.212657));
  vertices.push_back(fcl::Vector3<S>(-0.212629,0.000000,0.131412));
  vertices.push_back(fcl::Vector3<S>(-0.223567,0.000000,-0.111785));
  vertices.push_back(fcl::Vector3<S>(-0.237764,0.077232,0.000000));
  vertices.push_back(fcl::Vector3<S>(-0.131404,0.000000,-0.212657));
  vertices.push_back(fcl::Vector3<S>(0.000000,0.000000,-0.250000));
  vertices.push_back(fcl::Vector3<S>(0.106314,0.077232,-0.212657));
  vertices.push_back(fcl::Vector3<S>(0.180883,0.131412,-0.111785));
  vertices.push_back(fcl::Vector3<S>(0.146904,0.202232,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.172039,0.124951,0.131412));
  vertices.push_back(fcl::Vector3<S>(0.040589,0.124951,0.212657));
  vertices.push_back(fcl::Vector3<S>(-0.065678,0.202232,0.131412));
  vertices.push_back(fcl::Vector3<S>(-0.180883,0.131412,0.111785));
  vertices.push_back(fcl::Vector3<S>(-0.172039,0.124951,-0.131412));
  vertices.push_back(fcl::Vector3<S>(-0.146904,0.202232,0.000000));
  vertices.push_back(fcl::Vector3<S>(-0.040589,0.124951,-0.212657));
  vertices.push_back(fcl::Vector3<S>(0.065678,0.202232,-0.131412));
  vertices.push_back(fcl::Vector3<S>(0.000000,0.250000,0.000000));
  vertices.push_back(fcl::Vector3<S>(0.069076,0.212657,0.111785));
  vertices.push_back(fcl::Vector3<S>(-0.069076,0.212657,-0.111785));

  std::vector<int> faces = {3,0,1,2,3,0,2,3,3,0,3,4,3,0,4,5,3,0,5,6,3,0,6,1,3,1,7,8,3,1,8,2,
                            3,1,6,9,3,1,9,7,3,2,8,10,3,2,10,3,3,3,10,11,3,3,11,12,3,3,12,4,3,4,12,13,
                            3,4,13,14,3,4,14,5,3,5,14,15,3,5,15,6,3,6,15,16,3,6,16,9,3,7,9,17,3,7,17,18,
                            3,7,18,19,3,7,19,8,3,8,19,20,3,8,20,10,3,9,16,17,3,10,20,21,3,10,21,11,3,11,21,22,
                            3,11,22,23,3,11,23,12,3,12,23,13,3,13,24,14,3,13,23,25,3,13,25,24,3,14,26,15,3,14,24,26,
                            3,15,27,16,3,15,26,27,3,16,28,17,3,16,27,28,3,17,28,29,3,17,29,18,3,18,30,31,3,18,31,19,
                            3,18,29,30,3,19,31,20,3,20,31,32,3,20,32,21,3,21,32,22,3,22,32,33,3,22,33,34,3,22,34,23,
                            3,23,34,25,3,24,25,35,3,24,35,26,3,25,34,36,3,25,36,35,3,26,35,37,3,26,37,27,3,27,37,28,
                            3,28,37,38,3,28,38,29,3,29,38,30,3,30,39,40,3,30,40,31,3,30,38,39,3,31,40,32,3,32,40,33,
                            3,33,40,39,3,33,39,36,3,33,36,34,3,35,41,37,3,35,36,41,3,36,39,41,3,37,41,38,3,38,41,39};

  CollisionGeometryPtr_t convexGeometry1 (new fcl::Convex<S>(vertices.size(), vertices.data(), 80, faces.data()));
  CollisionGeometryPtr_t convexGeometry2 (new fcl::Convex<S>(vertices.size(), vertices.data(), 80, faces.data()));

  // Enable computation of nearest points
  fcl::DistanceRequest<S> distanceRequest (true, true);
  fcl::DistanceResult<S> distanceResult;

  distanceRequest.gjk_solver_type = solver_type;
  distanceRequest.distance_tolerance = solver_tolerance;

  fcl::Transform3<S> tf1, tf2;

  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////

  tf1.setIdentity();
  tf2.setIdentity();
  tf2.translation()(0) = 0.2;

  fcl::CollisionObject<S> convex1 (convexGeometry1, tf1);
  fcl::CollisionObject<S> convex2 (convexGeometry2, tf2);

  // test distance
  fcl::distance (&convex1, &convex2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, -0.27552, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.23776, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[1][0], -0.03776, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], distanceResult.nearest_points[1][1], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);

  ///////////////////////////////////////////////////////
  // Test object distance (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////
  distanceResult.clear();
  tf2.setIdentity();
  tf2.translation()(0) = 1;

  convex1 = fcl::CollisionObject<S>(convexGeometry1, tf1);
  convex2 = fcl::CollisionObject<S>(convexGeometry2, tf2);

  // test distance
  fcl::distance (&convex1, &convex2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, 0.52448, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.23776, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], 0.76224, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], distanceResult.nearest_points[1][1], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);

  //////////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Vertex to Vertex)
  //////////////////////////////////////////////////////////////////////
  distanceResult.clear();
  tf2.setIdentity();
  tf2.translation()(1) = 0.2;

  convex1 = fcl::CollisionObject<S>(convexGeometry1, tf1);
  convex2 = fcl::CollisionObject<S>(convexGeometry2, tf2);

  // test distance
  fcl::distance (&convex1, &convex2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, -0.25, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], 0.25, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[1][1], -0.05, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], distanceResult.nearest_points[1][0], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);

  ///////////////////////////////////////////////////////////
  // Test object distance (Closest Feature Vertex to Vertex)
  ///////////////////////////////////////////////////////////
  distanceResult.clear();
  tf2.setIdentity();
  tf2.translation()(1) = 1.0;

  convex1 = fcl::CollisionObject<S>(convexGeometry1, tf1);
  convex2 = fcl::CollisionObject<S>(convexGeometry2, tf2);

  // test distance
  fcl::distance (&convex1, &convex2, distanceRequest, distanceResult);

  EXPECT_NEAR(distanceResult.min_distance, 0.5, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][1], 0.25, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[1][1], 0.75, test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][0], distanceResult.nearest_points[1][0], test_tolerance);
  EXPECT_NEAR(distanceResult.nearest_points[0][2], distanceResult.nearest_points[1][2], test_tolerance);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, convex_sphere_sphere_ccd)
{
  test_convex_sphere_sphere<double>(fcl::GJKSolverType::GST_LIBCCD, 1e-6, 1e-4);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, convex_sphere_sphere_indep)
{
  test_convex_sphere_sphere<double>(fcl::GJKSolverType::GST_INDEP, 1e-8, 1e-4);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, convex_face_normal_ccd)
{
  test_convex_face_normal<double>(fcl::GJKSolverType::GST_LIBCCD, 1e-6, 1e-4);
}

// TODO: This test is planned to be fixed later
//GTEST_TEST(FCL_GEOMETRIC_SHAPES, cconvex_face_normal_indep)
//{
//  test_convex_face_normal<double>(fcl::GJKSolverType::GST_INDEP, 1e-8, 1e-4);
//}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, box_cylinder_ccd)
{
  test_box_cylinder<double>(fcl::GJKSolverType::GST_LIBCCD, 1e-6, 1e-4);
}

GTEST_TEST(FCL_GEOMETRIC_SHAPES, box_cylinder_indep)
{
  test_box_cylinder<double>(fcl::GJKSolverType::GST_INDEP, 1e-8, 1e-4);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
