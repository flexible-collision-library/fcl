#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/ccd/motion.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/ccd/conservative_advancement.h"
#include <iostream>

namespace fcl
{

template<typename GJKSolver>
ConservativeAdvancementFunctionMatrix<GJKSolver>& getConservativeAdvancementFunctionLookTable()
{
  static ConservativeAdvancementFunctionMatrix<GJKSolver> table;
  return table;
}

MotionBasePtr getMotionBase(const Transform3f& tf_beg, const Transform3f& tf_end, CCDMotionType motion_type)
{
  switch(motion_type)
  {
  case CCDM_TRANS:
    return MotionBasePtr(new TranslationMotion(tf_beg, tf_end));
    break;
  case CCDM_LINEAR:
    return MotionBasePtr(new InterpMotion(tf_beg, tf_end));
    break;
  case CCDM_SCREW:
    return MotionBasePtr(new ScrewMotion(tf_beg, tf_end));
    break;
  case CCDM_SPLINE:
    return MotionBasePtr(new SplineMotion(tf_beg, tf_end));
    break;
  default:
    return MotionBasePtr();
  }
}


FCL_REAL continuousCollideNaive(const CollisionGeometry* o1, const MotionBase* motion1,
                                const CollisionGeometry* o2, const MotionBase* motion2,
                                const ContinuousCollisionRequest& request,
                                ContinuousCollisionResult& result)
{
  std::size_t n_iter = std::min(request.num_max_iterations, (std::size_t)ceil(1 / request.toc_err));
  Transform3f cur_tf1, cur_tf2;
  for(std::size_t i = 0; i < n_iter; ++i)
  {
    FCL_REAL t = i / (FCL_REAL) (n_iter - 1);
    motion1->integrate(t);
    motion2->integrate(t);
    
    motion1->getCurrentTransform(cur_tf1);
    motion2->getCurrentTransform(cur_tf2);

    CollisionRequest c_request;
    CollisionResult c_result;

    if(collide(o1, cur_tf1, o2, cur_tf2, c_request, c_result))
    {
      result.is_collide = true;
      result.time_of_contact = t;
      result.contact_tf1 = cur_tf1;
      result.contact_tf2 = cur_tf2;
      return t;
    }
  }

  result.is_collide = false;
  result.time_of_contact = FCL_REAL(1);
  return result.time_of_contact;
}

namespace details
{
template<typename BV>
FCL_REAL continuousCollideBVHPolynomial(const CollisionGeometry* o1_, const TranslationMotion* motion1,
                                        const CollisionGeometry* o2_, const TranslationMotion* motion2,
                                        const ContinuousCollisionRequest& request,
                                        ContinuousCollisionResult& result)
{
  const BVHModel<BV>* o1__ = static_cast<const BVHModel<BV>*>(o1_);
  const BVHModel<BV>* o2__ = static_cast<const BVHModel<BV>*>(o2_);

  // ugly, but lets do it now.
  BVHModel<BV>* o1 = const_cast<BVHModel<BV>*>(o1__);
  BVHModel<BV>* o2 = const_cast<BVHModel<BV>*>(o2__);
  std::vector<Vec3f> new_v1(o1->num_vertices);
  std::vector<Vec3f> new_v2(o2->num_vertices);

  for(std::size_t i = 0; i < new_v1.size(); ++i)
    new_v1[i] = o1->vertices[i] + motion1->getVelocity();
  for(std::size_t i = 0; i < new_v2.size(); ++i)
    new_v2[i] = o2->vertices[i] + motion2->getVelocity();

  o1->beginUpdateModel();
  o1->updateSubModel(new_v1);
  o1->endUpdateModel(true, true);

  o2->beginUpdateModel();
  o2->updateSubModel(new_v2);
  o2->endUpdateModel(true, true);

  MeshContinuousCollisionTraversalNode<BV> node;
  CollisionRequest c_request;

  motion1->integrate(0);
  motion2->integrate(0);
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);
  if(!initialize<BV>(node, *o1, tf1, *o2, tf2, c_request))
    return -1.0;

  collide(&node);

  result.is_collide = (node.pairs.size() > 0);
  result.time_of_contact = node.time_of_contact;

  if(result.is_collide)
  {
    motion1->integrate(node.time_of_contact);
    motion2->integrate(node.time_of_contact);
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    result.contact_tf1 = tf1;
    result.contact_tf2 = tf2;
  }

  return result.time_of_contact;
}

}

FCL_REAL continuousCollideBVHPolynomial(const CollisionGeometry* o1, const TranslationMotion* motion1,
                                        const CollisionGeometry* o2, const TranslationMotion* motion2,
                                        const ContinuousCollisionRequest& request,
                                        ContinuousCollisionResult& result)
{
  switch(o1->getNodeType())
  {
  case BV_AABB:
    if(o2->getNodeType() == BV_AABB)
      return details::continuousCollideBVHPolynomial<AABB>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_OBB:
    if(o2->getNodeType() == BV_OBB)
      return details::continuousCollideBVHPolynomial<OBB>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_RSS:
    if(o2->getNodeType() == BV_RSS)
      return details::continuousCollideBVHPolynomial<RSS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_kIOS:
    if(o2->getNodeType() == BV_kIOS)
      return details::continuousCollideBVHPolynomial<kIOS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_OBBRSS:
    if(o2->getNodeType() == BV_OBBRSS)
      return details::continuousCollideBVHPolynomial<OBBRSS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP16:
    if(o2->getNodeType() == BV_KDOP16)
      return details::continuousCollideBVHPolynomial<KDOP<16> >(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP18:
    if(o2->getNodeType() == BV_KDOP18)
      return details::continuousCollideBVHPolynomial<KDOP<18> >(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP24:
    if(o2->getNodeType() == BV_KDOP24)
      return details::continuousCollideBVHPolynomial<KDOP<24> >(o1, motion1, o2, motion2, request, result);
    break;
  default:
    ;
  }

  std::cerr << "Warning: BV type not supported by polynomial solver CCD" << std::endl;

  return -1;
}

namespace details
{

template<typename NarrowPhaseSolver>
FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1,
                                                  const CollisionGeometry* o2, const MotionBase* motion2,
                                                  const NarrowPhaseSolver* nsolver_,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)
{
  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();

  const ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>& looktable = getConservativeAdvancementFunctionLookTable<NarrowPhaseSolver>();

  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();

  FCL_REAL res = -1;
  
  if(!looktable.conservative_advancement_matrix[node_type1][node_type2])
  {
    std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
  }
  else
  {
    res = looktable.conservative_advancement_matrix[node_type1][node_type2](o1, motion1, o2, motion2, nsolver, request, result);
  }

  if(!nsolver_)
    delete nsolver;

  if(result.is_collide)
  {
    motion1->integrate(result.time_of_contact);
    motion2->integrate(result.time_of_contact);

    Transform3f tf1, tf2;
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    result.contact_tf1 = tf1;
    result.contact_tf2 = tf2;
  }

  return res;
}

}


FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1,
                                                  const CollisionGeometry* o2, const MotionBase* motion2,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      GJKSolver_libccd solver;
      return details::continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
    }
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return details::continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
    }
  default:
    return -1;
  }
}

  
FCL_REAL continuousCollide(const CollisionGeometry* o1, const MotionBase* motion1,
                           const CollisionGeometry* o2, const MotionBase* motion2,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  switch(request.ccd_solver_type)
  {
  case CCDC_NAIVE:
    return continuousCollideNaive(o1, motion1,
                                  o2, motion2,
                                  request,
                                  result);
    break;
  case CCDC_CONSERVATIVE_ADVANCEMENT:
    return continuousCollideConservativeAdvancement(o1, motion1,
                                                    o2, motion2,
                                                    request,
                                                    result);
    break;
  case CCDC_RAY_SHOOTING:
    if(o1->getObjectType() == OT_GEOM && o2->getObjectType() == OT_GEOM && request.ccd_motion_type == CCDM_TRANS)
    {
      
    }
    else
      std::cerr << "Warning! Invalid continuous collision setting" << std::endl;
    break;
  case CCDC_POLYNOMIAL_SOLVER:
    if(o1->getObjectType() == OT_BVH && o2->getObjectType() == OT_BVH && request.ccd_motion_type == CCDM_TRANS)
    {
      return continuousCollideBVHPolynomial(o1, (const TranslationMotion*)motion1,
                                            o2, (const TranslationMotion*)motion2,
                                            request, result);
    }
    else
      std::cerr << "Warning! Invalid continuous collision checking" << std::endl;
    break;
  default:
    std::cerr << "Warning! Invalid continuous collision setting" << std::endl;
  }

  return -1;
}

FCL_REAL continuousCollide(const CollisionGeometry* o1, const Transform3f& tf1_beg, const Transform3f& tf1_end,
                           const CollisionGeometry* o2, const Transform3f& tf2_beg, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  MotionBasePtr motion1 = getMotionBase(tf1_beg, tf1_end, request.ccd_motion_type);
  MotionBasePtr motion2 = getMotionBase(tf2_beg, tf2_end, request.ccd_motion_type);

  return continuousCollide(o1, motion1.get(), o2, motion2.get(), request, result);
}


FCL_REAL continuousCollide(const CollisionObject* o1, const Transform3f& tf1_end,
                           const CollisionObject* o2, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  return continuousCollide(o1->collisionGeometry().get(), o1->getTransform(), tf1_end,
                           o2->collisionGeometry().get(), o2->getTransform(), tf2_end,
                           request, result);
}


FCL_REAL collide(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2,
                 const ContinuousCollisionRequest& request,
                 ContinuousCollisionResult& result)
{
  return continuousCollide(o1->collisionGeometry().get(), o1->getMotion(),
                           o2->collisionGeometry().get(), o2->getMotion(),
                           request, result);
}

}
