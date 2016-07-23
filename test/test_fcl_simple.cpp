#include <gtest/gtest.h>

#include "fcl/intersect.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl_resources/config.h"
#include <boost/filesystem.hpp>
#include <sstream>
#include "fcl/math/vec_nf.h"
#include "fcl/math/sampling.h"

using namespace fcl;

static FCL_REAL epsilon = 1e-6;

static bool approx(FCL_REAL x, FCL_REAL y)
{
  return std::abs(x - y) < epsilon;
}



template<std::size_t N>
double distance_Vecnf(const Vecnf<N>& a, const Vecnf<N>& b)
{
  double d = 0;
  for(std::size_t i = 0; i < N; ++i)
    d += (a[i] - b[i]) * (a[i] - b[i]);

  return d;
}


TEST(FCL_SIMPLE, Vec_nf_test)
{
  Vecnf<4> a;
  Vecnf<4> b;
  for(std::size_t i = 0; i < a.dim(); ++i)
    a[i] = i;
  for(std::size_t i = 0; i < b.dim(); ++i)
    b[i] = 1;

  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << a + b << std::endl;
  std::cout << a - b << std::endl;
  std::cout << (a -= b) << std::endl;
  std::cout << (a += b) << std::endl;
  std::cout << a * 2 << std::endl;
  std::cout << a / 2 << std::endl;
  std::cout << (a *= 2) << std::endl;
  std::cout << (a /= 2) << std::endl;
  std::cout << a.dot(b) << std::endl;

  Vecnf<8> c = combine(a, b);
  std::cout << c << std::endl;

  Vecnf<4> upper, lower;
  for(int i = 0; i < 4; ++i)
    upper[i] = 1;

  Vecnf<4> aa(std::vector<FCL_REAL>({1,2}));
  std::cout << aa << std::endl;

  SamplerR<4> sampler(lower, upper);
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler.sample() << std::endl;

  // Disabled broken test lines. Please see #25.
  // SamplerSE2 sampler2(0, 1, -1, 1);
  // for(std::size_t i = 0; i < 10; ++i)
  //   std::cout << sampler2.sample() << std::endl;

  SamplerSE3Euler sampler3(Vec3f(0, 0, 0), Vec3f(1, 1, 1));
  for(std::size_t i = 0; i < 10; ++i)
    std::cout << sampler3.sample() << std::endl;
  
}


TEST(FCL_SIMPLE, projection_test_line)
{
  Vec3f v1(0, 0, 0);
  Vec3f v2(2, 0, 0);
    
  Vec3f p(1, 0, 0);
  Project::ProjectResult res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
    
  p = Vec3f(-1, 0, 0);
  res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 1));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));

  p = Vec3f(3, 0, 0);
  res = Project::projectLine(v1, v2, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 1));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));

}

TEST(FCL_SIMPLE, projection_test_triangle)
{
  Vec3f v1(0, 0, 1);
  Vec3f v2(0, 1, 0);
  Vec3f v3(1, 0, 0);

  Vec3f p(1, 1, 1);
  Project::ProjectResult res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, 4/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  
  p = Vec3f(0, 0, 1.5);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));

  p = Vec3f(1.5, 0, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1));

  p = Vec3f(0, 1.5, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));
  EXPECT_TRUE(approx(res.parameterization[2], 0));

  p = Vec3f(1, 1, 0);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));

  p = Vec3f(1, 0, 1);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));

  p = Vec3f(0, 1, 1);
  res = Project::projectTriangle(v1, v2, v3, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0.5));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
}

TEST(FCL_SIMPLE, projection_test_tetrahedron)
{
  Vec3f v1(0, 0, 1);
  Vec3f v2(0, 1, 0);
  Vec3f v3(1, 0, 0);
  Vec3f v4(1, 1, 1);

  Vec3f p(0.5, 0.5, 0.5);
  Project::ProjectResult res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 15);
  EXPECT_TRUE(approx(res.sqr_distance, 0));
  EXPECT_TRUE(approx(res.parameterization[0], 0.25));
  EXPECT_TRUE(approx(res.parameterization[1], 0.25));
  EXPECT_TRUE(approx(res.parameterization[2], 0.25));
  EXPECT_TRUE(approx(res.parameterization[3], 0.25));

  p = Vec3f(0, 0, 0);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 7);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(0, 1, 1);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 11);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vec3f(1, 1, 0);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 14);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vec3f(1, 0, 1);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 13);
  EXPECT_TRUE(approx(res.sqr_distance, 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[0], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1/3.0));
  EXPECT_TRUE(approx(res.parameterization[3], 1/3.0));

  p = Vec3f(1.5, 1.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 8);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 1));

  p = Vec3f(1.5, -0.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 4);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 1));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(-0.5, -0.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 1);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 1));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(-0.5, 1.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 2);
  EXPECT_TRUE(approx(res.sqr_distance, 0.75));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 1));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(0.5, -0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 5);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(0.5, 1.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 10);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));

  p = Vec3f(1.5, 0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 12);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));
    
  p = Vec3f(-0.5, 0.5, 0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 3);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

  p = Vec3f(0.5, 0.5, 1.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 9);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0.5));
  EXPECT_TRUE(approx(res.parameterization[1], 0));
  EXPECT_TRUE(approx(res.parameterization[2], 0));
  EXPECT_TRUE(approx(res.parameterization[3], 0.5));
    
  p = Vec3f(0.5, 0.5, -0.5);
  res = Project::projectTetrahedra(v1, v2, v3, v4, p);
  EXPECT_TRUE(res.encode == 6);
  EXPECT_TRUE(approx(res.sqr_distance, 0.25));
  EXPECT_TRUE(approx(res.parameterization[0], 0));
  EXPECT_TRUE(approx(res.parameterization[1], 0.5));
  EXPECT_TRUE(approx(res.parameterization[2], 0.5));
  EXPECT_TRUE(approx(res.parameterization[3], 0));

}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
