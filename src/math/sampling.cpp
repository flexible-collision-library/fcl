#include "fcl/math/sampling.h"
#include <mutex>
#include <chrono>

namespace fcl
{

/// The seed the user asked for (cannot be 0)
static std::uint_fast32_t userSetSeed = 0;
	
/// Flag indicating whether the first seed has already been generated or not
static bool firstSeedGenerated = false;
	
/// The value of the first seed
static std::uint_fast32_t firstSeedValue = 0;
	
/// Compute the first seed to be used; this function should be called only once
static std::uint_fast32_t firstSeed()
{
  static std::mutex fsLock;
  std::unique_lock<std::mutex> slock(fsLock);
		
  if(firstSeedGenerated)
    return firstSeedValue;
			
  if(userSetSeed != 0)
    firstSeedValue = userSetSeed;
  else firstSeedValue = (std::uint_fast32_t)std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now() - std::chrono::system_clock::time_point()).count();
  firstSeedGenerated = true;
		
  return firstSeedValue;
}
	
/// We use a different random number generator for the seeds of the
/// Other random generators. The root seed is from the number of
/// nano-seconds in the current time.
static std::uint_fast32_t nextSeed()
{
  static std::mutex rngMutex;
  std::unique_lock<std::mutex> slock(rngMutex);
  static std::ranlux24_base sGen;
  static std::uniform_int_distribution<> sDist(1, 1000000000);
  return sDist(sGen);
}
	
std::uint_fast32_t RNG::getSeed()
{
  return firstSeed();
}
	
void RNG::setSeed(std::uint_fast32_t seed)
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
                 normalDist_(0, 1)
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
  double x0 = uniDist_(generator_);
  double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
  double t1 = 2.0 * constants::pi * uniDist_(generator_), t2 = 2.0 * constants::pi * uniDist_(generator_);
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
  value[0] = constants::pi * (2.0 * uniDist_(generator_) - 1.0);
  value[1] = acos(1.0 - 2.0 * uniDist_(generator_)) - constants::pi / 2.0;
  value[2] = constants::pi * (2.0 * uniDist_(generator_) - 1.0);
}
	
void RNG::disk(double r_min, double r_max, double& x, double& y)
{
  double a = uniform01();
  double b = uniform01();
  double r = std::sqrt(a * r_max * r_max + (1 - a) * r_min * r_min);
  double theta = 2 * constants::pi * b;
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
  double phi = 2 * constants::pi * c;
		
  double costheta = std::cos(theta);
  double sintheta = std::sin(theta);
  double cosphi = std::cos(phi);
  double sinphi = std::sin(phi);
  x = r * costheta;
  y = r * sintheta * cosphi;
  z = r * sintheta * sinphi;
}

}
