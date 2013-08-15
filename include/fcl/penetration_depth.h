#ifndef FCL_PENETRATION_DEPTH_H
#define FCL_PENETRATION_DEPTH_H

#include "fcl/math/vec_3f.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/learning/classifier.h"
#include "fcl/knn/nearest_neighbors.h"
#include <boost/mem_fn.hpp>
#include <iostream>

namespace fcl
{

typedef double (*TransformDistance)(const Transform3f&, const Transform3f&);

class DefaultTransformDistancer
{
public:
  const CollisionGeometry* o1;
  const CollisionGeometry* o2;

  FCL_REAL rot_x_weight, rot_y_weight, rot_z_weight;

  DefaultTransformDistancer() : o1(NULL), o2(NULL)
  {
    rot_x_weight = rot_y_weight = rot_z_weight = 1;
  }

  DefaultTransformDistancer(const CollisionGeometry* o2_)
  {
    o2 = o2_;
    init();
  }

  DefaultTransformDistancer(const CollisionGeometry* o1_, const CollisionGeometry* o2_)
  {
    o1 = o1_;
    o2 = o2_;
    init();
  }

  void init()
  {
    rot_x_weight = rot_y_weight = rot_z_weight = 1;

    if(o2)
    {
      FCL_REAL V = o2->computeVolume();
      Matrix3f C = o2->computeMomentofInertiaRelatedToCOM();
      FCL_REAL eigen_values[3];
      Vec3f eigen_vectors[3];
      eigen(C, eigen_values, eigen_vectors);
      rot_x_weight = eigen_values[0] * 4 / V;
      rot_y_weight = eigen_values[1] * 4 / V;
      rot_z_weight = eigen_values[2] * 4 / V;

      // std::cout << rot_x_weight << " " << rot_y_weight << " " << rot_z_weight << std::endl;
    }
  }

  void printRotWeight() const
  {
    std::cout << rot_x_weight << " " << rot_y_weight << " " << rot_z_weight << std::endl;
  }

  double distance(const Transform3f& tf1, const Transform3f& tf2)
  {
    Transform3f tf;
    relativeTransform(tf1, tf2, tf);

    const Quaternion3f& quat = tf.getQuatRotation();
    const Vec3f& trans = tf.getTranslation();

    FCL_REAL d = rot_x_weight * quat[1] * quat[1] + rot_y_weight * quat[2] * quat[2] + rot_z_weight * quat[3] * quat[3] + trans[0] * trans[0] + trans[1] * trans[1] + trans[2] * trans[2];
    return d;
  }
};

static DefaultTransformDistancer default_transform_distancer;

static NearestNeighbors<Transform3f>::DistanceFunction default_transform_distance_func = boost::bind(&DefaultTransformDistancer::distance, &default_transform_distancer, _1, _2);



/// @brief precompute a learning model for penetration computation
std::vector<Transform3f> penetrationDepthModelLearning(const CollisionObject* o1,
                                                       const CollisionObject* o2,
                                                       PenetrationDepthType pd_type,
                                                       void* classifier,
                                                       std::size_t n_samples,
                                                       FCL_REAL margin,
                                                       KNNSolverType knn_solver_type,
                                                       std::size_t knn_k,
                                                       NearestNeighbors<Transform3f>::DistanceFunction distance_func = default_transform_distance_func);




/// @brief penetration depth computation between two in-collision objects
FCL_REAL penetrationDepth(const CollisionGeometry* o1, const Transform3f& tf1,
                          const CollisionGeometry* o2, const Transform3f& tf2,
                          const PenetrationDepthRequest& request,
                          PenetrationDepthResult& result);

FCL_REAL penetrationDepth(const CollisionObject* o1,
                          const CollisionObject* o2,
                          const PenetrationDepthRequest& request,
                          PenetrationDepthResult& result);

}

#endif
