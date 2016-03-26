#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <iostream>
#include <fcl/collision.h>

using namespace std;
using namespace fcl;

int main(int argc, char** argv) 
{
  std::shared_ptr<Box> box0(new Box(1,1,1));
  std::shared_ptr<Box> box1(new Box(1,1,1));
//  GJKSolver_indep solver;
  GJKSolver_libccd solver;
  Vec3f contact_points;
  FCL_REAL penetration_depth;
  Vec3f normal;

  Transform3f tf0, tf1;
  tf0.setIdentity();
  tf0.setTranslation(Vec3f(.9,0,0));
  tf0.setQuatRotation(Quaternion3f(.6, .8, 0, 0));
  tf1.setIdentity();



  bool res = solver.shapeIntersect(*box0, tf0, *box1, tf1, &contact_points, &penetration_depth, &normal);

  cout << "contact points: " << contact_points << endl;
  cout << "pen depth: " << penetration_depth << endl;
  cout << "normal: " << normal << endl;
  cout << "result: " << res << endl;
  
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  fcl::CollisionResult result;
  fcl::CollisionRequest request(num_max_contacts,
                                enable_contact);


  CollisionObject co0(box0, tf0);
  CollisionObject co1(box1, tf1);

  fcl::collide(&co0, &co1, request, result);
  vector<Contact> contacts;
  result.getContacts(contacts);

  cout << contacts.size() << " contacts found" << endl;
  for(const Contact &contact : contacts) {
    cout << "position: " << contact.pos << endl;
  }
}