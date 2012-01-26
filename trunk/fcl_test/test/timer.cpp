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

#include "timer.h"
#include <cstddef>


Timer::Timer()
{
#ifdef _WIN32
        QueryPerformanceFrequency(&frequency);
        startCount.QuadPart = 0;
        endCount.QuadPart = 0;
#else
        startCount.tv_sec = startCount.tv_usec = 0;
        endCount.tv_sec = endCount.tv_usec = 0;
#endif

        stopped = 0;
        startTimeInMicroSec = 0;
        endTimeInMicroSec = 0;
}


Timer::~Timer()
{
}


void Timer::start()
{
        stopped = 0; // reset stop flag
#ifdef _WIN32
        QueryPerformanceCounter(&startCount);
#else
        gettimeofday(&startCount, NULL);
#endif
}


void Timer::stop()
{
        stopped = 1; // set timer stopped flag

#ifdef _WIN32
        QueryPerformanceCounter(&endCount);
#else
        gettimeofday(&endCount, NULL);
#endif
}


double Timer::getElapsedTimeInMicroSec()
{
#ifdef _WIN32
        if(!stopped)
                QueryPerformanceCounter(&endCount);

        startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
        endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);
#else
        if(!stopped)
                gettimeofday(&endCount, NULL);

        startTimeInMicroSec = (startCount.tv_sec * 1000000.0) + startCount.tv_usec;
        endTimeInMicroSec = (endCount.tv_sec * 1000000.0) + endCount.tv_usec;
#endif

        return endTimeInMicroSec - startTimeInMicroSec;
}


double Timer::getElapsedTimeInMilliSec()
{
        return this->getElapsedTimeInMicroSec() * 0.001;
}


double Timer::getElapsedTimeInSec()
{
        return this->getElapsedTimeInMicroSec() * 0.000001;
}


double Timer::getElapsedTime()
{
        return this->getElapsedTimeInMilliSec();
}
