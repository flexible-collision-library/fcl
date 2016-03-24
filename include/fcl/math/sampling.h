#ifndef FCL_MATH_SAMPLING_H
#define FCL_MATH_SAMPLING_H

#include <random>
#include <cassert>
#include "fcl/math/constants.h"
#include "fcl/math/vec_nf.h"
#include "fcl/math/transform.h"

namespace fcl
{

/// @brief Random number generation. An instance of this class
/// cannot be used by multiple threads at once (member functions
/// are not const). However, the constructor is thread safe and
/// different instances can be used safely in any number of
/// threads. It is also guaranteed that all created instances will
/// have a different random seed.

class RNG
{
public:
	
  /// @brief Constructor. Always sets a different random seed
  RNG();
		
  /// @brief Generate a random real between 0 and 1
  double uniform01()
  {
    return uniDist_(generator_);
  }
		
  /// @brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound)
  double uniformReal(double lower_bound, double upper_bound)
  {
    assert(lower_bound <= upper_bound);
    return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
  }
		
  /// @brief Generate a random integer within given bounds: [\e lower_bound, \e upper_bound]
  int uniformInt(int lower_bound, int upper_bound)
  {
    int r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
    return (r > upper_bound) ? upper_bound : r;
  }
		
  /// @brief Generate a random boolean
  bool uniformBool()
  {
    return uniDist_(generator_) <= 0.5;
  }
		
  /// @brief Generate a random real using a normal distribution with mean 0 and variance 1
  double gaussian01()
  {
    return normalDist_(generator_);
  }
		
  /// @brief Generate a random real using a normal distribution with given mean and variance
  double gaussian(double mean, double stddev)
  {
    return normalDist_(generator_) * stddev + mean;
  }
		
  /// @brief Generate a random real using a half-normal distribution. The value is within specified bounds [\e r_min, \e r_max], but with a bias towards \e r_max. The function is implemended using a Gaussian distribution with mean at \e r_max - \e r_min. The distribution is 'folded' around \e r_max axis towards \e r_min. The variance of the distribution is (\e r_max - \e r_min) / \e focus. The higher the focus, the more probable it is that generated numbers are close to \e r_max.
  double halfNormalReal(double r_min, double r_max, double focus = 3.0);
		
  /// @brief Generate a random integer using a half-normal distribution. The value is within specified bounds ([\e r_min, \e r_max]), but with a bias towards \e r_max. The function is implemented on top of halfNormalReal()
  int halfNormalInt(int r_min, int r_max, double focus = 3.0);
		
  /// @brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w) 
  void quaternion(double value[4]);
		
  /// @brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the range [-pi, pi). The computed value has the order (roll, pitch, yaw) */
  void   eulerRPY(double value[3]);
		
  /// @brief Uniform random sample on a disk with radius from r_min to r_max
  void disk(double r_min, double r_max, double& x, double& y);
		
  /// @brief Uniform random sample in a ball with radius from r_min to r_max
  void ball(double r_min, double r_max, double& x, double& y, double& z);
		
  /// @brief Set the seed for random number generation. Use this function to ensure the same sequence of random numbers is generated.
  static void setSeed(std::uint_fast32_t seed);
		
  /// @brief Get the seed used for random number generation. Passing the returned value to setSeed() at a subsequent execution of the code will ensure deterministic (repeatable) behaviour. Useful for debugging.
  static std::uint_fast32_t getSeed();
		
private:
	
  std::mt19937                     generator_;
  std::uniform_real_distribution<> uniDist_;
  std::normal_distribution<>       normalDist_;
		
};

class SamplerBase
{
public:
  mutable RNG rng;
};

template<std::size_t N>
class SamplerR : public SamplerBase
{
public:
  SamplerR() {}

  SamplerR(const Vecnf<N>& lower_bound_,
           const Vecnf<N>& upper_bound_) : lower_bound(lower_bound_),
                                           upper_bound(upper_bound_)
  {
  }

  void setBound(const Vecnf<N>& lower_bound_,
                const Vecnf<N>& upper_bound_)
  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(Vecnf<N>& lower_bound_,
                Vecnf<N>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  Vecnf<N> sample() const
  {
    Vecnf<N> q;

    for(std::size_t i = 0; i < N; ++i)
    {
      q[i] = rng.uniformReal(lower_bound[i], upper_bound[i]);
    }

    return q;
  }

private:
  Vecnf<N> lower_bound;
  Vecnf<N> upper_bound;
};


class SamplerSE2 : public SamplerBase
{
public:
  SamplerSE2() {}

  SamplerSE2(const Vecnf<2>& lower_bound_,
             const Vecnf<2>& upper_bound_) : lower_bound(lower_bound_),
                                             upper_bound(upper_bound_)
  {}

  SamplerSE2(FCL_REAL x_min, FCL_REAL x_max,
             FCL_REAL y_min, FCL_REAL y_max) : lower_bound(std::vector<FCL_REAL>({x_min, y_min})),
                                               upper_bound(std::vector<FCL_REAL>({x_max, y_max}))
                                               
  {}


  void setBound(const Vecnf<2>& lower_bound_,
                const Vecnf<2>& upper_bound_)
  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void getBound(Vecnf<2>& lower_bound_,
                Vecnf<2>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }


  Vecnf<3> sample() const
  {
    Vecnf<3> q;
    q[0] = rng.uniformReal(lower_bound[0], lower_bound[1]);
    q[1] = rng.uniformReal(lower_bound[1], lower_bound[2]);
    q[2] = rng.uniformReal(-constants::pi, constants::pi);

    return q;
  }

protected:
  Vecnf<2> lower_bound;
  Vecnf<2> upper_bound;
};


class SamplerSE2_disk : public SamplerBase
{
public:
  SamplerSE2_disk() {}

  SamplerSE2_disk(FCL_REAL cx, FCL_REAL cy,
                  FCL_REAL r1, FCL_REAL r2,
                  FCL_REAL crefx, FCL_REAL crefy)
  {
    setBound(cx, cy, r1, r2, crefx, crefy);
  }

  void setBound(FCL_REAL cx, FCL_REAL cy,
                FCL_REAL r1, FCL_REAL r2,
                FCL_REAL crefx, FCL_REAL crefy)
  {
    c[0] = cx; c[1] = cy;
    cref[0] = crefx; cref[1] = crefy;
    r_min = r1;
    r_max = r2;
  }

  Vecnf<3> sample() const
  {
    Vecnf<3> q;
    FCL_REAL x, y;
    rng.disk(r_min, r_max, x, y);
    q[0] = x + c[0] - cref[0];
    q[1] = y + c[1] - cref[1];
    q[2] = rng.uniformReal(-constants::pi, constants::pi);

    return q;
  }

protected:
  FCL_REAL c[2];
  FCL_REAL cref[2];
  FCL_REAL r_min, r_max;
};

class SamplerSE3Euler : public SamplerBase
{
public:
  SamplerSE3Euler() {}

  SamplerSE3Euler(const Vecnf<3>& lower_bound_,
                  const Vecnf<3>& upper_bound_) : lower_bound(lower_bound_),
                                                  upper_bound(upper_bound_)
  {}

  SamplerSE3Euler(const Vec3f& lower_bound_,
                  const Vec3f& upper_bound_) : lower_bound(std::vector<FCL_REAL>({lower_bound_[0], lower_bound_[1], lower_bound_[2]})),
                                               upper_bound(std::vector<FCL_REAL>({upper_bound_[0], upper_bound_[1], upper_bound_[2]}))
  {}

  void setBound(const Vecnf<3>& lower_bound_,
                const Vecnf<3>& upper_bound_)

  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void setBound(const Vec3f& lower_bound_,
                const Vec3f& upper_bound_)
  {
    lower_bound = Vecnf<3>(std::vector<FCL_REAL>({lower_bound_[0], lower_bound_[1], lower_bound_[2]}));
    upper_bound = Vecnf<3>(std::vector<FCL_REAL>({upper_bound_[0], upper_bound_[1], upper_bound_[2]}));
  }

  void getBound(Vecnf<3>& lower_bound_,
                Vecnf<3>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  void getBound(Vec3f& lower_bound_,
                Vec3f& upper_bound_) const
  {
    lower_bound_ = Vec3f(lower_bound[0], lower_bound[1], lower_bound[2]);
    upper_bound_ = Vec3f(upper_bound[0], upper_bound[1], upper_bound[2]);
  }

  Vecnf<6> sample() const
  {
    Vecnf<6> q;
    q[0] = rng.uniformReal(lower_bound[0], upper_bound[0]);
    q[1] = rng.uniformReal(lower_bound[1], upper_bound[1]);
    q[2] = rng.uniformReal(lower_bound[2], upper_bound[2]);

    FCL_REAL s[4];
    rng.quaternion(s);

    Quaternion3f quat(s[0], s[1], s[2], s[3]);
    FCL_REAL a, b, c;
    quat.toEuler(a, b, c);

    q[3] = a;
    q[4] = b;
    q[5] = c;

    return q;
  }

protected:
  Vecnf<3> lower_bound;
  Vecnf<3> upper_bound;
  
};

class SamplerSE3Quat : public SamplerBase
{
public:
  SamplerSE3Quat() {}

  SamplerSE3Quat(const Vecnf<3>& lower_bound_,
                 const Vecnf<3>& upper_bound_) : lower_bound(lower_bound_),
                                                 upper_bound(upper_bound_)
  {}

  SamplerSE3Quat(const Vec3f& lower_bound_,
                 const Vec3f& upper_bound_) : lower_bound(std::vector<FCL_REAL>({lower_bound_[0], lower_bound_[1], lower_bound_[2]})),
                                              upper_bound(std::vector<FCL_REAL>({upper_bound_[0], upper_bound_[1], upper_bound_[2]}))
  {}


  void setBound(const Vecnf<3>& lower_bound_,
                const Vecnf<3>& upper_bound_)

  {
    lower_bound = lower_bound_;
    upper_bound = upper_bound_;
  }

  void setBound(const Vec3f& lower_bound_,
                const Vec3f& upper_bound_)
  {
    lower_bound = Vecnf<3>(std::vector<FCL_REAL>({lower_bound_[0], lower_bound_[1], lower_bound_[2]}));
    upper_bound = Vecnf<3>(std::vector<FCL_REAL>({upper_bound_[0], upper_bound_[1], upper_bound_[2]}));
  }


  void getBound(Vecnf<3>& lower_bound_,
                Vecnf<3>& upper_bound_) const
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  void getBound(Vec3f& lower_bound_,
                Vec3f& upper_bound_) const
  {
    lower_bound_ = Vec3f(lower_bound[0], lower_bound[1], lower_bound[2]);
    upper_bound_ = Vec3f(upper_bound[0], upper_bound[1], upper_bound[2]);
  }

  Vecnf<7> sample() const
  {
    Vecnf<7> q;
    q[0] = rng.uniformReal(lower_bound[0], upper_bound[0]);
    q[1] = rng.uniformReal(lower_bound[1], upper_bound[1]);
    q[2] = rng.uniformReal(lower_bound[2], upper_bound[2]);

    FCL_REAL s[4];
    rng.quaternion(s);

    q[3] = s[0];
    q[4] = s[1];
    q[5] = s[2];
    q[6] = s[3];
    return q;
  }

protected:
  Vecnf<3> lower_bound;
  Vecnf<3> upper_bound;
};

class SamplerSE3Euler_ball : public SamplerBase
{
public:
  SamplerSE3Euler_ball() {}

  SamplerSE3Euler_ball(FCL_REAL r_) : r(r_)
  {
  }

  void setBound(const FCL_REAL& r_)
  {
    r = r_;
  }
  
  void getBound(FCL_REAL& r_) const
  {
    r_ = r;
  }

  Vecnf<6> sample() const
  {
    Vecnf<6> q;
    FCL_REAL x, y, z;
    rng.ball(0, r, x, y, z);
    q[0] = x;
    q[1] = y;
    q[2] = z;

    FCL_REAL s[4];
    rng.quaternion(s);

    Quaternion3f quat(s[0], s[1], s[2], s[3]);
    FCL_REAL a, b, c;
    quat.toEuler(a, b, c);
    q[3] = a;
    q[4] = b;
    q[5] = c;
    
    return q;
  }

protected:
  FCL_REAL r;

};


class SamplerSE3Quat_ball : public SamplerBase
{
public:
  SamplerSE3Quat_ball() {}

  SamplerSE3Quat_ball(FCL_REAL r_) : r(r_)
  {}

  void setBound(const FCL_REAL& r_)
  {
    r = r_;
  }

  void getBound(FCL_REAL& r_) const
  {
    r_ = r;
  }

  Vecnf<7> sample() const
  {
    Vecnf<7> q;
    FCL_REAL x, y, z;
    rng.ball(0, r, x, y, z);
    q[0] = x;
    q[1] = y;
    q[2] = z;

    FCL_REAL s[4];
    rng.quaternion(s);

    q[3] = s[0];
    q[4] = s[1];
    q[5] = s[2];
    q[6] = s[3];
    return q;
  }

protected:
  FCL_REAL r;
};



}

#endif
