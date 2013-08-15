#define BOOST_TEST_MODULE "FCL_GLOBAL_PENETRATION"
#include <boost/test/unit_test.hpp>
#include "libsvm_classifier.h"
#include "fcl/penetration_depth.h"
#include <boost/filesystem.hpp>
#include "fcl_resources/config.h"
#include "test_fcl_utility.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"




using namespace fcl;



BOOST_AUTO_TEST_CASE(global_penetration_test)
{
  LibSVMClassifier<6> classifier;
  
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  BVHModel<OBBRSS>* m1 = new BVHModel<OBBRSS>();
  BVHModel<OBBRSS>* m2 = new BVHModel<OBBRSS>();
  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();
  m2->beginModel();
  m2->addSubModel(p2, t2);
  m2->endModel();

  std::cout << m1->computeVolume() << std::endl;
  std::cout << m2->computeVolume() << std::endl;

  std::cout << m1->computeMomentofInertia() << std::endl;
  std::cout << m2->computeMomentofInertia() << std::endl;

  std::cout << m1->computeMomentofInertiaRelatedToCOM() << std::endl;
  std::cout << m2->computeMomentofInertiaRelatedToCOM() << std::endl;

  Transform3f id;

  CollisionObject o1(boost::shared_ptr<CollisionGeometry>(m1), id);
  CollisionObject o2(boost::shared_ptr<CollisionGeometry>(m2), id);

  default_transform_distancer = DefaultTransformDistancer(o2.getCollisionGeometry());

  std::size_t KNN_K = 10;
  std::vector<Transform3f> contact_vectors = penetrationDepthModelLearning(&o1, &o2, PDT_GENERAL_EULER, &classifier, 10000, 0, KNN_GNAT, KNN_K);

  classifier.save("model.txt");

  PenetrationDepthRequest request(&classifier, default_transform_distance_func);
  request.contact_vectors = contact_vectors;

  PenetrationDepthResult result;
  penetrationDepth(o1.getCollisionGeometry(), Transform3f(),
                   o2.getCollisionGeometry(), Transform3f(),
                   request, result);

  std::cout << result.pd_value << std::endl;

}
