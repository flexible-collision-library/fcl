/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2014, Willow Garage, Inc.
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

/** @author Ioan Sucan */

#include "fcl/common/detail/profiler.h"

namespace fcl {
namespace detail {

//==============================================================================
Profiler& Profiler::Instance(void)
{
  static Profiler p(true, false);
  return p;
}

//==============================================================================
Profiler::Profiler(bool printOnDestroy, bool autoStart)
  : running_(false), printOnDestroy_(printOnDestroy)
{
  if (autoStart)
    start();
}

//==============================================================================
Profiler::~Profiler()
{
  if (printOnDestroy_ && !data_.empty())
    status();
}

//==============================================================================
void Profiler::Start()
{
  Instance().start();
}

//==============================================================================
void Profiler::Stop()
{
  Instance().stop();
}

//==============================================================================
void Profiler::Clear()
{
  Instance().clear();
}

//==============================================================================
void Profiler::start(void)
{
  lock_.lock();
  if (!running_)
  {
    tinfo_.set();
    running_ = true;
  }
  lock_.unlock();
}

//==============================================================================
void Profiler::stop(void)
{
  lock_.lock();
  if (running_)
  {
    tinfo_.update();
    running_ = false;
  }
  lock_.unlock();
}

//==============================================================================
void Profiler::clear(void)
{
  lock_.lock();
  data_.clear();
  tinfo_ = TimeInfo();
  if (running_)
    tinfo_.set();
  lock_.unlock();
}

//==============================================================================
void Profiler::Event(const std::string& name, const unsigned int times)
{
  Instance().event(name, times);
}

//==============================================================================
void Profiler::event(const std::string &name, const unsigned int times)
{
  lock_.lock();
  data_[std::this_thread::get_id()].events[name] += times;
  lock_.unlock();
}

//==============================================================================
void Profiler::Average(const std::string& name, const double value)
{
  Instance().average(name, value);
}

//==============================================================================
void Profiler::average(const std::string &name, const double value)
{
  lock_.lock();
  AvgInfo &a = data_[std::this_thread::get_id()].avg[name];
  a.total += value;
  a.totalSqr += value*value;
  a.parts++;
  lock_.unlock();
}

//==============================================================================
void Profiler::Begin(const std::string& name)
{
  Instance().begin(name);
}

//==============================================================================
void Profiler::End(const std::string& name)
{
  Instance().end(name);
}

//==============================================================================
void Profiler::begin(const std::string &name)
{
  lock_.lock();
  data_[std::this_thread::get_id()].time[name].set();
  lock_.unlock();
}

//==============================================================================
void Profiler::end(const std::string &name)
{
  lock_.lock();
  data_[std::this_thread::get_id()].time[name].update();
  lock_.unlock();
}

//==============================================================================
void Profiler::Status(std::ostream& out, bool merge)
{
  Instance().status(out, merge);
}

//==============================================================================
void Profiler::status(std::ostream &out, bool merge)
{
  stop();
  lock_.lock();
  printOnDestroy_ = false;

  out << std::endl;
  out << " *** Profiling statistics. Total counted time : " << time::seconds(tinfo_.total) << " seconds" << std::endl;

  if (merge)
  {
    PerThread combined;
    for (std::map<std::thread::id, PerThread>::const_iterator it = data_.begin() ; it != data_.end() ; ++it)
    {
      for (std::map<std::string, unsigned long int>::const_iterator iev = it->second.events.begin() ; iev != it->second.events.end(); ++iev)
        combined.events[iev->first] += iev->second;
      for (std::map<std::string, AvgInfo>::const_iterator iavg = it->second.avg.begin() ; iavg != it->second.avg.end(); ++iavg)
      {
        combined.avg[iavg->first].total += iavg->second.total;
        combined.avg[iavg->first].totalSqr += iavg->second.totalSqr;
        combined.avg[iavg->first].parts += iavg->second.parts;
      }
      for (std::map<std::string, TimeInfo>::const_iterator itm = it->second.time.begin() ; itm != it->second.time.end(); ++itm)
      {
        TimeInfo &tc = combined.time[itm->first];
        tc.total = tc.total + itm->second.total;
        tc.parts = tc.parts + itm->second.parts;
        if (tc.shortest > itm->second.shortest)
          tc.shortest = itm->second.shortest;
        if (tc.longest < itm->second.longest)
          tc.longest = itm->second.longest;
      }
    }
    printThreadInfo(out, combined);
  }
  else
    for (std::map<std::thread::id, PerThread>::const_iterator it = data_.begin() ; it != data_.end() ; ++it)
    {
      out << "Thread " << it->first << ":" << std::endl;
      printThreadInfo(out, it->second);
    }
  lock_.unlock();
}

//==============================================================================
bool Profiler::running() const
{
  return running_;
}

//==============================================================================
bool Profiler::Running()
{
  return Instance().running();
}

//==============================================================================
struct FCL_EXPORT dataIntVal
{
  std::string       name;
  unsigned long int value;
};

//==============================================================================
struct FCL_EXPORT SortIntByValue
{
  bool operator()(const dataIntVal &a, const dataIntVal &b) const
  {
    return a.value > b.value;
  }
};

//==============================================================================
struct FCL_EXPORT dataDoubleVal
{
  std::string  name;
  double       value;
};

//==============================================================================
struct FCL_EXPORT SortDoubleByValue
{
  bool operator()(const dataDoubleVal &a, const dataDoubleVal &b) const
  {
    return a.value > b.value;
  }
};

//==============================================================================
void Profiler::printThreadInfo(std::ostream &out, const PerThread &data)
{
  double total = time::seconds(tinfo_.total);

  std::vector<detail::dataIntVal> events;
  for (std::map<std::string, unsigned long int>::const_iterator iev = data.events.begin() ; iev != data.events.end() ; ++iev)
  {
    detail::dataIntVal next = {iev->first, iev->second};
    events.push_back(next);
  }
  std::sort(events.begin(), events.end(), SortIntByValue());
  if (!events.empty())
    out << "Events:" << std::endl;
  for (unsigned int i = 0 ; i < events.size() ; ++i)
    out << events[i].name << ": " << events[i].value << std::endl;

  std::vector<detail::dataDoubleVal> avg;
  for (std::map<std::string, AvgInfo>::const_iterator ia = data.avg.begin() ; ia != data.avg.end() ; ++ia)
  {
    detail::dataDoubleVal next = {ia->first, ia->second.total / (double)ia->second.parts};
    avg.push_back(next);
  }
  std::sort(avg.begin(), avg.end(), SortDoubleByValue());
  if (!avg.empty())
    out << "Averages:" << std::endl;
  for (unsigned int i = 0 ; i < avg.size() ; ++i)
  {
    const AvgInfo &a = data.avg.find(avg[i].name)->second;
    out << avg[i].name << ": " << avg[i].value << " (stddev = " <<
      std::sqrt(std::abs(a.totalSqr - (double)a.parts * avg[i].value * avg[i].value) / ((double)a.parts - 1.)) << ")" << std::endl;
  }

  std::vector<detail::dataDoubleVal> time;

  for (std::map<std::string, TimeInfo>::const_iterator itm = data.time.begin() ; itm != data.time.end() ; ++itm)
  {
    detail::dataDoubleVal next = {itm->first, time::seconds(itm->second.total)};
    time.push_back(next);
  }

  std::sort(time.begin(), time.end(), detail::SortDoubleByValue());
  if (!time.empty())
    out << "Blocks of time:" << std::endl;

  double unaccounted = total;
  for (unsigned int i = 0 ; i < time.size() ; ++i)
  {
    const TimeInfo &d = data.time.find(time[i].name)->second;

    double tS = time::seconds(d.shortest);
    double tL = time::seconds(d.longest);
    out << time[i].name << ": " << time[i].value << "s (" << (100.0 * time[i].value/total) << "%), ["
        << tS << "s --> " << tL << " s], " << d.parts << " parts";
    if (d.parts > 0)
      out << ", " << (time::seconds(d.total) / (double)d.parts) << " s on average";
    out << std::endl;
    unaccounted -= time[i].value;
  }
  out << "Unaccounted time : " << unaccounted;
  if (total > 0.0)
    out << " (" << (100.0 * unaccounted / total) << " %)";
  out << std::endl;

  out << std::endl;
}

//==============================================================================
Profiler::TimeInfo::TimeInfo()
  : total(time::seconds(0.)),
    shortest(time::duration::max()),
    longest(time::duration::min()),
    parts(0)
{
  // Do nothing
}

//==============================================================================
void Profiler::TimeInfo::set()
{
  start = time::now();
}

//==============================================================================
void Profiler::TimeInfo::update()
{
  const time::duration &dt = time::now() - start;

  if (dt > longest)
    longest = dt;

  if (dt < shortest)
    shortest = dt;

  total = total + dt;
  ++parts;
}

//==============================================================================
Profiler::ScopedStart::ScopedStart(Profiler& prof)
  : prof_(prof), wasRunning_(prof_.running())
{
  if (!wasRunning_)
    prof_.start();
}

//==============================================================================
Profiler::ScopedStart::~ScopedStart()
{
  if (!wasRunning_)
    prof_.stop();
}

//==============================================================================
Profiler::ScopedBlock::ScopedBlock(const std::string& name, Profiler& prof)
  : name_(name), prof_(prof)
{
  prof_.begin(name);
}

//==============================================================================
Profiler::ScopedBlock::~ScopedBlock()
{
  prof_.end(name_);
}

} // namespace detail
} // namespace fcl
