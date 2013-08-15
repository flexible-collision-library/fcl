#include "fcl/math/sampling.h"
#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>

namespace fcl
{

/// The seed the user asked for (cannot be 0)
static boost::uint32_t userSetSeed = 0;
	
/// Flag indicating whether the first seed has already been generated or not
static bool firstSeedGenerated = false;
	
/// The value of the first seed
static boost::uint32_t firstSeedValue = 0;
	
/// Compute the first seed to be used; this function should be called only once
static boost::uint32_t firstSeed()
{
  static boost::mutex fsLock;
  boost::mutex::scoped_lock slock(fsLock);
		
  if(firstSeedGenerated)
    return firstSeedValue;
			
  if(userSetSeed != 0)
    firstSeedValue = userSetSeed;
  else firstSeedValue = (boost::uint32_t)(boost::posix_time::microsec_clock::universal_time() - boost::posix_time::ptime(boost::date_time::min_date_time)).total_microseconds();
  firstSeedGenerated = true;
		
  return firstSeedValue;
}
	
/// We use a different random number generator for the seeds of the
/// Other random generators. The root seed is from the number of
/// nano-seconds in the current time.
static boost::uint32_t nextSeed()
{
  static boost::mutex rngMutex;
  boost::mutex::scoped_lock slock(rngMutex);
  static boost::lagged_fibonacci607 sGen(firstSeed());
  static boost::uniform_int<> sDist(1, 1000000000);
  static boost::variate_generator<boost::lagged_fibonacci607&, boost::uniform_int<> > s(sGen, sDist);
  return s();
}
	
boost::uint32_t RNG::getSeed()
{
  return firstSeed();
}
	
void RNG::setSeed(boost::uint32_t seed)
{
  if(firstSeedGenerated)
  {
    std::cerr << "Random number generation already started. Changing seed now will not lead to deterministic sampling." << std::endl;
  }
  if(seed == 0)
  {
    std::cerr << "Random generator seed cannot be 0. Using 1 instead." << std::endl;
    userSetSeed = 1;
  }
  else
    userSetSeed = seed;
}
	
RNG::RNG() : generator_(nextSeed()),
                 uniDist_(0, 1),
                 normalDist_(0, 1),
                 uni_(generator_, uniDist_),
                 normal_(generator_, normalDist_)
{
}
	
double RNG::halfNormalReal(double r_min, double r_max, double focus)
{
  assert(r_min <= r_max);
		
  const double mean = r_max - r_min;
  double v = gaussian(mean, mean / focus);
		
  if(v > mean) v = 2.0 * mean - v;
  double r = v >= 0.0 ? v + r_min : r_min;
  return r > r_max ? r_max : r;
}
	
int RNG::halfNormalInt(int r_min, int r_max, double focus)
{
  int r = (int)floor(halfNormalReal((double)r_min, (double)(r_max) + 1.0, focus));
  return (r > r_max) ? r_max : r;
}
	
// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void RNG::quaternion(double value[4])
{
  double x0 = uni_();
  double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
  double t1 = 2.0 * boost::math::constants::pi<double>() * uni_(), t2 = 2.0 * boost::math::constants::pi<double>() * uni_();
  double c1 = cos(t1), s1 = sin(t1);
  double c2 = cos(t2), s2 = sin(t2);
  value[0] = s1 * r1;
  value[1] = c1 * r1;
  value[2] = s2 * r2;
  value[3] = c2 * r2;
}
	
// From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, by James Kuffner, ICRA 2004
void RNG::eulerRPY(double value[3])
{
  value[0] = boost::math::constants::pi<double>() * (2.0 * uni_() - 1.0);
  value[1] = acos(1.0 - 2.0 * uni_()) - boost::math::constants::pi<double>() / 2.0;
  value[2] = boost::math::constants::pi<double>() * (2.0 * uni_() - 1.0);
}
	
void RNG::disk(double r_min, double r_max, double& x, double& y)
{
  double a = uniform01();
  double b = uniform01();
  double r = std::sqrt(a * r_max * r_max + (1 - a) * r_min * r_min);
  double theta = 2 * boost::math::constants::pi<double>() * b;
  x = r * std::cos(theta);
  y = r * std::sin(theta);
}
	
void RNG::ball(double r_min, double r_max, double& x, double& y, double& z)
{
  double a = uniform01();
  double b = uniform01();
  double c = uniform01();
  double r = std::pow(a * r_max * r_max * r_max + (1 - a) * r_min * r_min * r_min, 1 / 3.0);
  double theta = std::acos(1 - 2 * b);
  double phi = 2 * boost::math::constants::pi<double>() * c;
		
  double costheta = std::cos(theta);
  double sintheta = std::sin(theta);
  double cosphi = std::cos(phi);
  double sinphi = std::sin(phi);
  x = r * costheta;
  y = r * sintheta * cosphi;
  z = r * sintheta * sinphi;
}

}
