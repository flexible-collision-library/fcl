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

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

/** Timer for CPU
 */
class Timer
{
public:
        Timer();
        ~Timer();

        void   start();                             ///< start timer
        void   stop();                              ///< stop the timer
        double getElapsedTime();                    ///< get elapsed time in milli-second
        double getElapsedTimeInSec();               ///< get elapsed time in second (same as getElapsedTime)
        double getElapsedTimeInMilliSec();          ///< get elapsed time in milli-second
        double getElapsedTimeInMicroSec();          ///< get elapsed time in micro-second

private:
        double startTimeInMicroSec;                 ///< starting time in micro-second
        double endTimeInMicroSec;                   ///< ending time in micro-second
        int    stopped;                             ///< stop flag
#ifdef _WIN32
        LARGE_INTEGER frequency;                    ///< ticks per second
        LARGE_INTEGER startCount;
        LARGE_INTEGER endCount;
#else
        timeval startCount;
        timeval endCount;
#endif
};
