#include "test_fcl_utility.h"
#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/distance.h"
#include <cstdio>
#include <cstddef>
#include <fstream>

namespace fcl
{


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


FCL_REAL rand_interval(FCL_REAL rmin, FCL_REAL rmax)
{
  FCL_REAL t = rand() / ((FCL_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
          FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
          Vec3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}


void saveOBJFile(const char* filename, std::vector<Vec3f>& points, std::vector<Triangle>& triangles)
{
  std::ofstream os(filename);
  if(!os)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  for(std::size_t i = 0; i < points.size(); ++i)
  {
    os << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
  }

  for(std::size_t i = 0; i < triangles.size(); ++i)
  {
    os << "f " << triangles[i][0] + 1 << " " << triangles[i][1] + 1 << " " << triangles[i][2] + 1 << std::endl;
  }

  os.close();
}


void eulerToMatrix(FCL_REAL a, FCL_REAL b, FCL_REAL c, Matrix3f& R)
{
  FCL_REAL c1 = cos(a);
  FCL_REAL c2 = cos(b);
  FCL_REAL c3 = cos(c);
  FCL_REAL s1 = sin(a);
  FCL_REAL s2 = sin(b);
  FCL_REAL s3 = sin(c);

  R.setValue(c1 * c2, - c2 * s1, s2,
             c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
             s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3);
}

void generateRandomTransform(FCL_REAL extents[6], Transform3f& transform)
{
  FCL_REAL x = rand_interval(extents[0], extents[3]);
  FCL_REAL y = rand_interval(extents[1], extents[4]);
  FCL_REAL z = rand_interval(extents[2], extents[5]);

  const FCL_REAL pi = 3.1415926;
  FCL_REAL a = rand_interval(0, 2 * pi);
  FCL_REAL b = rand_interval(0, 2 * pi);
  FCL_REAL c = rand_interval(0, 2 * pi);

  Matrix3f R;
  eulerToMatrix(a, b, c, R);
  Vec3f T(x, y, z);
  transform.setTransform(R, T);
}


void generateRandomTransforms(FCL_REAL extents[6], std::vector<Transform3f>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);

    {
      Matrix3f R;
      eulerToMatrix(a, b, c, R);
      Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}


void generateRandomTransforms(FCL_REAL extents[6], FCL_REAL delta_trans[3], FCL_REAL delta_rot, std::vector<Transform3f>& transforms, std::vector<Transform3f>& transforms2, std::size_t n)
{
  transforms.resize(n);
  transforms2.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);

    {
      Matrix3f R;
      eulerToMatrix(a, b, c, R);
      Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }

    FCL_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
    FCL_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
    FCL_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

    FCL_REAL deltaa = rand_interval(-delta_rot, delta_rot);
    FCL_REAL deltab = rand_interval(-delta_rot, delta_rot);
    FCL_REAL deltac = rand_interval(-delta_rot, delta_rot);

    {
      Matrix3f R;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R);
      Vec3f T(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setTransform(R, T);
    }
  }
}

void generateRandomTransform_ccd(FCL_REAL extents[6], std::vector<Transform3f>& transforms, std::vector<Transform3f>& transforms2, FCL_REAL delta_trans[3], FCL_REAL delta_rot, std::size_t n,
                                 const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                                 const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2)
{
  transforms.resize(n);
  transforms2.resize(n);

  for(std::size_t i = 0; i < n;)
  {
    FCL_REAL x = rand_interval(extents[0], extents[3]);
    FCL_REAL y = rand_interval(extents[1], extents[4]);
    FCL_REAL z = rand_interval(extents[2], extents[5]);

    const FCL_REAL pi = 3.1415926;
    FCL_REAL a = rand_interval(0, 2 * pi);
    FCL_REAL b = rand_interval(0, 2 * pi);
    FCL_REAL c = rand_interval(0, 2 * pi);


    Matrix3f R;
    eulerToMatrix(a, b, c, R);
    Vec3f T(x, y, z);    
    Transform3f tf(R, T);

    std::vector<std::pair<int, int> > results;
    {
      transforms[i] = tf;

      FCL_REAL deltax = rand_interval(-delta_trans[0], delta_trans[0]);
      FCL_REAL deltay = rand_interval(-delta_trans[1], delta_trans[1]);
      FCL_REAL deltaz = rand_interval(-delta_trans[2], delta_trans[2]);

      FCL_REAL deltaa = rand_interval(-delta_rot, delta_rot);
      FCL_REAL deltab = rand_interval(-delta_rot, delta_rot);
      FCL_REAL deltac = rand_interval(-delta_rot, delta_rot);

      Matrix3f R2;
      eulerToMatrix(a + deltaa, b + deltab, c + deltac, R2);
      Vec3f T2(x + deltax, y + deltay, z + deltaz);
      transforms2[i].setTransform(R2, T2);
      ++i;
    }
  }
}

bool defaultCollisionFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_)
{
  CollisionData* cdata = static_cast<CollisionData*>(cdata_);
  const CollisionRequest& request = cdata->request;
  CollisionResult& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

bool defaultDistanceFunction(CollisionObject* o1, CollisionObject* o2, void* cdata_, FCL_REAL& dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  const DistanceRequest& request = cdata->request;
  DistanceResult& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  distance(o1, o2, request, result);
  
  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

bool defaultContinuousCollisionFunction(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata_)
{
  ContinuousCollisionData* cdata = static_cast<ContinuousCollisionData*>(cdata_);
  const ContinuousCollisionRequest& request = cdata->request;
  ContinuousCollisionResult& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  return cdata->done;
}

bool defaultContinuousDistanceFunction(ContinuousCollisionObject* o1, ContinuousCollisionObject* o2, void* cdata_, FCL_REAL& dist)
{
  return true;
}

std::string getNodeTypeName(NODE_TYPE node_type)
{
  if (node_type == BV_UNKNOWN)
    return std::string("BV_UNKNOWN");
  else if (node_type == BV_AABB)
    return std::string("BV_AABB");
  else if (node_type == BV_OBB)
    return std::string("BV_OBB");
  else if (node_type == BV_RSS)
    return std::string("BV_RSS");
  else if (node_type == BV_kIOS)
    return std::string("BV_kIOS");
  else if (node_type == BV_OBBRSS)
    return std::string("BV_OBBRSS");
  else if (node_type == BV_KDOP16)
    return std::string("BV_KDOP16");
  else if (node_type == BV_KDOP18)
    return std::string("BV_KDOP18");
  else if (node_type == BV_KDOP24)
    return std::string("BV_KDOP24");
  else if (node_type == GEOM_BOX)
    return std::string("GEOM_BOX");
  else if (node_type == GEOM_SPHERE)
    return std::string("GEOM_SPHERE");
  else if (node_type == GEOM_ELLIPSOID)
    return std::string("GEOM_ELLIPSOID");
  else if (node_type == GEOM_CAPSULE)
    return std::string("GEOM_CAPSULE");
  else if (node_type == GEOM_CONE)
    return std::string("GEOM_CONE");
  else if (node_type == GEOM_CYLINDER)
    return std::string("GEOM_CYLINDER");
  else if (node_type == GEOM_CONVEX)
    return std::string("GEOM_CONVEX");
  else if (node_type == GEOM_PLANE)
    return std::string("GEOM_PLANE");
  else if (node_type == GEOM_HALFSPACE)
    return std::string("GEOM_HALFSPACE");
  else if (node_type == GEOM_TRIANGLE)
    return std::string("GEOM_TRIANGLE");
  else if (node_type == GEOM_OCTREE)
    return std::string("GEOM_OCTREE");
  else
    return std::string("invalid");
}

std::string getGJKSolverName(GJKSolverType solver_type)
{
  if (solver_type == GST_LIBCCD)
    return std::string("libccd");
  else if (solver_type == GST_INDEP)
    return std::string("built-in");
  else
    return std::string("invalid");
}

}
