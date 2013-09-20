#include "fcl/penetration_depth.h"
#include "fcl/learning/classifier.h"
#include "fcl/math/sampling.h"
#include "fcl/collision.h"
#include "fcl/exception.h"

#include "fcl/knn/nearest_neighbors_linear.h"
#include "fcl/knn/nearest_neighbors_GNAT.h"
#include "fcl/knn/nearest_neighbors_sqrtapprox.h"
#if FCL_HAVE_FLANN
#include "fcl/knn/nearest_neighbors_flann.h"
#endif

#include "fcl/ccd/motion.h"
#include "fcl/continuous_collision.h"

namespace fcl
{

std::vector<Transform3f> penetrationDepthModelLearning(const CollisionObject* o1,
                                                       const CollisionObject* o2,
                                                       PenetrationDepthType pd_type,
                                                       void* classifier_,
                                                       std::size_t n_samples,
                                                       FCL_REAL margin,
                                                       KNNSolverType knn_solver_type,
                                                       std::size_t knn_k,
                                                       NearestNeighbors<Transform3f>::DistanceFunction distance_func)
{
  std::vector<Transform3f> support_transforms_positive;
  std::vector<Transform3f> support_transforms_negative;
  
  switch(pd_type)
  {
  case PDT_TRANSLATIONAL:
    {
      SVMClassifier<3>* classifier = static_cast<SVMClassifier<3>*>(classifier_);
      SamplerR<3> sampler;
      Vecnf<3> lower_bound, upper_bound;
      AABB aabb1 = o1->getAABB();
      AABB aabb2 = o2->getAABB();
      lower_bound[0] = aabb1.min_[0] - aabb2.max_[0] - margin;
      lower_bound[1] = aabb1.min_[1] - aabb2.max_[1] - margin;
      lower_bound[2] = aabb1.min_[2] - aabb2.max_[2] - margin;
      upper_bound[0] = aabb1.max_[0] - aabb2.min_[0] + margin;
      upper_bound[1] = aabb1.max_[1] - aabb2.min_[1] + margin;
      upper_bound[2] = aabb1.max_[2] - aabb2.min_[2] + margin;
      
      sampler.setBound(lower_bound, upper_bound);

      std::vector<Item<3> > data(n_samples);
      for(std::size_t i = 0; i < n_samples; ++i)
      {
        Vecnf<3> q = sampler.sample();
        Transform3f tf(o2->getQuatRotation(), Vec3f(q[0], q[1], q[2]));
        
        CollisionRequest request;
        CollisionResult result;
        
        int n_collide = collide(o1->getCollisionGeometry(), o1->getTransform(),
                                o2->getCollisionGeometry(), tf, request, result);

        data[i] = Item<3>(q, (n_collide > 0));
      }

      // classifier.setScaler(Scaler<3>(lower_bound, upper_bound));
      classifier->setScaler(computeScaler(data));

      classifier->learn(data);

      std::vector<Item<3> > svs = classifier->getSupportVectors();
      for(std::size_t i = 0; i < svs.size(); ++i)
      {
        const Vecnf<3>& q = svs[i].q;
        if(svs[i].label)
          support_transforms_positive.push_back(Transform3f(o2->getQuatRotation(), Vec3f(q[0], q[1], q[2])));
        else
          support_transforms_negative.push_back(Transform3f(o2->getQuatRotation(), Vec3f(q[0], q[1], q[2])));
      }
      
      break;
    }
  case PDT_GENERAL_EULER:
    {
      SVMClassifier<6>* classifier = static_cast<SVMClassifier<6>*>(classifier_);
      SamplerSE3Euler sampler;

      FCL_REAL r1 = o1->getCollisionGeometry()->aabb_radius;
      FCL_REAL r2 = o2->getCollisionGeometry()->aabb_radius;
      Vec3f c1 = o1->getCollisionGeometry()->aabb_center;
      Vec3f c2 = o2->getCollisionGeometry()->aabb_center;

      FCL_REAL r = r1 + r2 + margin;
      sampler.setBound(Vec3f(-r, -r, -r), Vec3f(r, r, r));

      std::vector<Item<6> > data(n_samples);
      for(std::size_t i = 0; i < n_samples; ++i)
      {
        Vecnf<6> q = sampler.sample();
        Quaternion3f quat;
        quat.fromEuler(q[3], q[4], q[5]);
        Transform3f tf(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2]));
        CollisionRequest request;
        CollisionResult result;
        
        int n_collide = collide(o1->getCollisionGeometry(), Transform3f(),
                                o2->getCollisionGeometry(), tf, request, result);

        data[i] = Item<6>(q, (n_collide > 0));        
      }
      
      classifier->setScaler(computeScaler(data));

      classifier->learn(data);

      std::vector<Item <6> > svs = classifier->getSupportVectors();
      for(std::size_t i = 0; i < svs.size(); ++i)
      {
        const Vecnf<6>& q = svs[i].q;
        Quaternion3f quat;
        quat.fromEuler(q[3], q[4], q[5]);
        if(svs[i].label)
          support_transforms_positive.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
        else
          support_transforms_negative.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
      }
      
      break;
    }
  case PDT_GENERAL_QUAT:
    {
      SVMClassifier<7>* classifier = static_cast<SVMClassifier<7>*>(classifier_);
      SamplerSE3Quat sampler;

      FCL_REAL r1 = o1->getCollisionGeometry()->aabb_radius;
      FCL_REAL r2 = o2->getCollisionGeometry()->aabb_radius;
      Vec3f c1 = o1->getCollisionGeometry()->aabb_center;
      Vec3f c2 = o2->getCollisionGeometry()->aabb_center;

      FCL_REAL r = r1 + r2 + margin;
      sampler.setBound(Vec3f(-r, -r, -r), Vec3f(r, r, r));
      
      std::vector<Item<7> > data(n_samples);
      for(std::size_t i = 0; i < n_samples; ++i)
      {
        Vecnf<7> q = sampler.sample();
        
        Quaternion3f quat(q[3], q[4], q[5], q[6]);
        Transform3f tf(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2]));
        
        CollisionRequest request;
        CollisionResult result;
        
        int n_collide = collide(o1->getCollisionGeometry(), Transform3f(),
                                o2->getCollisionGeometry(), tf, request, result);

        data[i] = Item<7>(q, (n_collide > 0));
      }
      
      classifier->setScaler(computeScaler(data));

      classifier->learn(data);

      std::vector<Item<7> > svs = classifier->getSupportVectors();
      for(std::size_t i = 0; i < svs.size(); ++i)
      {
        const Vecnf<7>& q = svs[i].q;
        Quaternion3f quat(q[3], q[4], q[5], q[6]);
        if(svs[i].label)
          support_transforms_positive.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
        else
          support_transforms_negative.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
      }
      
      break;
    }
  case PDT_GENERAL_EULER_BALL:
    {
      SVMClassifier<6>* classifier = static_cast<SVMClassifier<6>*>(classifier_);
      SamplerSE3Euler_ball sampler;

      FCL_REAL r1 = o1->getCollisionGeometry()->aabb_radius;
      FCL_REAL r2 = o2->getCollisionGeometry()->aabb_radius;
      Vec3f c1 = o1->getCollisionGeometry()->aabb_center;
      Vec3f c2 = o2->getCollisionGeometry()->aabb_center;

      sampler.setBound(r1 + r2 + margin);
      
      std::vector<Item<6> > data(n_samples);
      for(std::size_t i = 0; i < n_samples; ++i)
      {
        Vecnf<6> q = sampler.sample();

        Quaternion3f quat;
        quat.fromEuler(q[3], q[4], q[5]);
        Transform3f tf(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2]));
        
        CollisionRequest request;
        CollisionResult result;
        
        int n_collide = collide(o1->getCollisionGeometry(), Transform3f(),
                                o2->getCollisionGeometry(), tf, request, result);

        data[i] = Item<6>(q, (n_collide > 0));
      }

      classifier->setScaler(computeScaler(data));

      classifier->learn(data);

      std::vector<Item<6> > svs = classifier->getSupportVectors();
      for(std::size_t i = 0; i < svs.size(); ++i)
      {
        const Vecnf<6>& q = svs[i].q;
        Quaternion3f quat;
        quat.fromEuler(q[3], q[4], q[5]);
        if(svs[i].label)
          support_transforms_positive.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
        else
          support_transforms_negative.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
      }
            
      break;
    }
  case PDT_GENERAL_QUAT_BALL:
    {
      SVMClassifier<7>* classifier = static_cast<SVMClassifier<7>*>(classifier_);
      SamplerSE3Quat_ball sampler;

      FCL_REAL r1 = o1->getCollisionGeometry()->aabb_radius;
      FCL_REAL r2 = o2->getCollisionGeometry()->aabb_radius;
      Vec3f c1 = o1->getCollisionGeometry()->aabb_center;
      Vec3f c2 = o2->getCollisionGeometry()->aabb_center;

      sampler.setBound(r1 + r2 + margin);
      
      std::vector<Item<7> > data(n_samples);
      for(std::size_t i = 0; i < n_samples; ++i)
      {
        Vecnf<7> q = sampler.sample();

        Quaternion3f quat(q[3], q[4], q[5], q[6]);
        Transform3f tf(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2]));
        
        CollisionRequest request;
        CollisionResult result;
        
        int n_collide = collide(o1->getCollisionGeometry(), Transform3f(),
                                o2->getCollisionGeometry(), tf, request, result);

        data[i] = Item<7>(q, (n_collide > 0));
      }

      classifier->setScaler(computeScaler(data));

      classifier->learn(data);

      std::vector<Item<7> > svs = classifier->getSupportVectors();
      for(std::size_t i = 0; i < svs.size(); ++i)
      {
        const Vecnf<7>& q = svs[i].q;
        Quaternion3f quat(q[3], q[4], q[5], q[6]);
        if(svs[i].label)
          support_transforms_positive.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
        else
          support_transforms_negative.push_back(Transform3f(quat, -quat.transform(c2) + c1 + Vec3f(q[0], q[1], q[2])));
      }
      
      break;
    }
  default:
    ;
  }

  // from support transforms compute contact transforms

  NearestNeighbors<Transform3f>* knn_solver_positive = NULL;
  NearestNeighbors<Transform3f>* knn_solver_negative = NULL;

  switch(knn_solver_type)
  {
  case KNN_LINEAR:
    knn_solver_positive = new NearestNeighborsLinear<Transform3f>();
    knn_solver_negative = new NearestNeighborsLinear<Transform3f>();
    break;
  case KNN_GNAT:
    knn_solver_positive = new NearestNeighborsGNAT<Transform3f>();
    knn_solver_negative = new NearestNeighborsGNAT<Transform3f>();
    break;
  case KNN_SQRTAPPROX:
    knn_solver_positive = new NearestNeighborsSqrtApprox<Transform3f>();
    knn_solver_negative = new NearestNeighborsSqrtApprox<Transform3f>();
    break;
  }


  knn_solver_positive->setDistanceFunction(distance_func);
  knn_solver_negative->setDistanceFunction(distance_func);
  
  knn_solver_positive->add(support_transforms_positive);
  knn_solver_negative->add(support_transforms_negative);

  std::vector<Transform3f> contact_vectors;

  for(std::size_t i = 0; i < support_transforms_positive.size(); ++i)
  {
    std::vector<Transform3f> nbh;
    knn_solver_negative->nearestK(support_transforms_positive[i], knn_k, nbh);

    for(std::size_t j = 0; j < nbh.size(); ++j)
    {
      ContinuousCollisionRequest request;
      request.ccd_motion_type = CCDM_LINEAR;
      request.num_max_iterations = 100;
      ContinuousCollisionResult result;
      continuousCollide(o1->getCollisionGeometry(), Transform3f(), Transform3f(),
                        o2->getCollisionGeometry(), nbh[j], support_transforms_positive[i],
                        request, result);

      //std::cout << result.time_of_contact << std::endl;

      contact_vectors.push_back(result.contact_tf2);
    }
  }

  for(std::size_t i = 0; i < support_transforms_negative.size(); ++i)
  {
    std::vector<Transform3f> nbh;
    knn_solver_positive->nearestK(support_transforms_negative[i], knn_k, nbh);

    for(std::size_t j = 0; j < nbh.size(); ++j)
    {
      ContinuousCollisionRequest request;
      request.ccd_motion_type = CCDM_LINEAR;
      request.num_max_iterations = 100;
      ContinuousCollisionResult result;
      continuousCollide(o1->getCollisionGeometry(), Transform3f(), Transform3f(),
                        o2->getCollisionGeometry(), support_transforms_negative[i], nbh[j],
                        request, result);

      //std::cout << result.time_of_contact << std::endl;

      contact_vectors.push_back(result.contact_tf2);
    }
  }

  delete knn_solver_positive;
  delete knn_solver_negative;

  return contact_vectors;
  
}



FCL_REAL penetrationDepth(const CollisionGeometry* o1, const Transform3f& tf1,
                          const CollisionGeometry* o2, const Transform3f& tf2,
                          const PenetrationDepthRequest& request,
                          PenetrationDepthResult& result)
{
  NearestNeighbors<Transform3f>* knn_solver = NULL;
  switch(request.knn_solver_type)
  {
  case KNN_LINEAR:
    knn_solver = new NearestNeighborsLinear<Transform3f>();
    break;
  case KNN_GNAT:
    knn_solver = new NearestNeighborsGNAT<Transform3f>();
    break;
  case KNN_SQRTAPPROX:
    knn_solver = new NearestNeighborsSqrtApprox<Transform3f>();
    break;
  }

  knn_solver->setDistanceFunction(request.distance_func);

  knn_solver->add(request.contact_vectors);

  Transform3f tf;
  relativeTransform2(tf1, tf2, tf);

  result.resolved_tf = tf1 * knn_solver->nearest(tf);
  result.pd_value = request.distance_func(result.resolved_tf, tf2);
  result.pd_value = std::sqrt(result.pd_value);

  delete knn_solver;

  return result.pd_value;
}

FCL_REAL penetrationDepth(const CollisionObject* o1,
                          const CollisionObject* o2,
                          const PenetrationDepthRequest& request,
                          PenetrationDepthResult& result)
{
  return penetrationDepth(o1->getCollisionGeometry(), o1->getTransform(),
                          o2->getCollisionGeometry(), o2->getTransform(),
                          request,
                          result);
}

}
