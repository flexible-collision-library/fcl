## FCL -- The Flexible Collision Library [![Build Status](https://travis-ci.org/flexible-collision-library/fcl.svg)](https://travis-ci.org/flexible-collision-library/fcl)

FCL is a library for performing three types of proximity queries on a pair of geometric models composed of triangles. 
 - Collision detection: detecting whether the two models overlap, and optionally, all of the triangles that overlap.
 - Distance computation: computing the minimum distance between a pair of models, i.e., the distance between the closest pair of points.
 - Tolerance verification: determining whether two models are closer or farther than a tolerance distance.
 - Continuous collision detection: detecting whether the two moving models overlap during the movement, and optionally, the time of contact.
 - Contact information: for collision detection and continuous collision detection, the contact information (including contact normals and contact points) can be returned optionally.

FCL has the following features
 - C++ interface, heavily use the boost
 - Compilable for either linux or win32 (both makefiles and Microsoft Visual projects can be generated using cmake)
 - No special topological constraints or adjacency information required for input models – all that is necessary is a list of the model's triangles
 - Supported different object shapes:
  + sphere
  + box
  + cone
  + cylinder
  + mesh
  + octree (optional, octrees are represented using the octomap library http://octomap.github.com)


## Installation

Before compiling FCL, please make sure boost and libccd (for collision checking between convex objects and is available here https://github.com/danfis/libccd) are installed. For libccd, make sure to compile from github version instead of the zip file from the webpage, because one bug fixing is not included in the zipped version.

Some optional libraries need to be installed for some optional capability of FCL. For octree collision, please install the octomap library from http://octomap.github.com. For global penetration depth, please install FLANN from https://github.com/mariusmuja/flann. For global penetration depth test, please install tinyxml.

CMakeLists.txt is used to generate makefiles in Linux or Visual studio projects in windows. In command line, run
``` cmake
mkdir build
cd build
cmake ..
```
Next, in linux, use make to compile the code. 

In windows, there will generate a visual studio project and then you can compile the code.

## Interfaces
Before starting the proximity computation, we need first to set the geometry and transform for the objects involving in computation. The geometry of an object is represented as a mesh soup, which can be set as follows:

```cpp
// set mesh triangles and vertice indices
std::vector<Vec3f> vertices;
std::vector<Triangle> triangles;
// code to set the vertices and triangles
...
// BVHModel is a template class for mesh geometry, for default OBBRSS template is used
typedef BVHModel<OBBRSS>* Model;
Model* model = new Model();
// add the mesh data into the BVHModel structure
model->beginModel();
model->addSubModel(vertices, triangles);
model->endModel();
```

The transform of an object includes the rotation and translation:
```cpp
// R and T are the rotation matrix and translation vector
Matrix3f R;
Vec3f T;
// code for setting R and T
...
// transform is configured according to R and T
Transform3f pose(R, T);
```


Given the geometry and the transform, we can also combine them together to obtain a collision object instance and here is an example:
```cpp
//geom and tf are the geometry and the transform of the object
BVHModel<OBBRSS>* geom = ...
Transform3f tf = ...
//Combine them together
CollisionObject* obj = new CollisionObject(geom, tf);
```

Once the objects are set, we can perform the proximity computation between them. All the proximity queries in FCL follow a common pipeline: first, set the query request data structure and then run the query function by using request as the input. The result is returned in a query result data structure. For example, for collision checking, we first set the CollisionRequest data structure, and then run the collision function:
```cpp
// Given two objects o1 and o2
CollisionObject* o1 = ...
CollisionObject* o2 = ...
// set the collision request structure, here we just use the default setting
CollisionRequest request;
// result will be returned via the collision result structure
CollisionResult result;
// perform collision test
collide(o1, o2, request, result);
```

By setting the collision request, the user can easily choose whether to return contact information (which is slower) or just return binary collision results (which is faster). 


For distance computation, the pipeline is almost the same:

```cpp
// Given two objects o1 and o2
CollisionObject* o1 = ...
CollisionObject* o2 = ...
// set the distance request structure, here we just use the default setting
DistanceRequest request;
// result will be returned via the collision result structure
DistanceResult result;
// perform distance test
distance(o1, o2, request, result);
```

For continuous collision, FCL requires the goal transform to be provided (the initial transform is included in the collision object data structure). Beside that, the pipeline is almost the same as distance/collision:

```cpp
// Given two objects o1 and o2
CollisionObject* o1 = ...
CollisionObject* o2 = ...
// The goal transforms for o1 and o2
Transform3f tf_goal_o1 = ...
Transform3f tf_goal_o2 = ...
// set the continuous collision request structure, here we just use the default setting
ContinuousCollisionRequest request;
// result will be returned via the continuous collision result structure
ContinuousCollisionResult result;
// perform continuous collision test
continuousCollide(o1, tf_goal_o1, o2, tf_goal_o2, request, result);
```

FCL supports broadphase collision/distance between two groups of objects and can avoid the n square complexity. For collision, broadphase algorithm can return all the collision pairs. For distance, it can return the pair with the minimum distance. FCL uses a CollisionManager structure to manage all the objects involving the collision or distance operations.
```cpp
// Initialize the collision manager for the first group of objects. 
// FCL provides various different implementations of CollisionManager.
// Generally, the DynamicAABBTreeCollisionManager would provide the best performance.
BroadPhaseCollisionManager* manager1 = new DynamicAABBTreeCollisionManager(); 
// Initialize the collision manager for the second group of objects.
BroadPhaseCollisionManager* manager2 = new DynamicAABBTreeCollisionManager();
// To add objects into the collision manager, using BroadPhaseCollisionManager::registerObject() function to add one object
std::vector<CollisionObject*> objects1 = ...
for(std::size_t i = 0; i < objects1.size(); ++i)
manager1->registerObject(objects1[i]);
// Another choose is to use BroadPhaseCollisionManager::registerObjects() function to add a set of objects
std::vector<CollisionObject*> objects2 = ...
manager2->registerObjects(objects2);
// In order to collect the information during broadphase, CollisionManager requires two settings: 
// a) a callback to collision or distance; 
// b) an intermediate data to store the information generated during the broadphase computation
// For a), FCL provides the default callbacks for both collision and distance.
// For b), FCL uses the CollisionData structure for collision and DistanceData structure for distance. CollisionData/DistanceData is just a container including both the CollisionRequest/DistanceRequest and CollisionResult/DistanceResult structures mentioned above.
CollisionData collision_data;
DistanceData distance_data;
// Setup the managers, which is related with initializing the broadphase acceleration structure according to objects input
manager1->setup();
manager2->setup();
// Examples for various queries
// 1. Collision query between two object groups and get collision numbers
manager2->collide(manager1, &collision_data, defaultCollisionFunction);
int n_contact_num = collision_data.result.numContacts(); 
// 2. Distance query between two object groups and get the minimum distance
manager2->distance(manager1, &distance_data, defaultDistanceFunction);
double min_distance = distance_data.result.min_distance;
// 3. Self collision query for group 1
manager1->collide(&collision_data, defaultCollisionFunction);
// 4. Self distance query for group 1
manager1->distance(&distance_data, defaultDistanceFunction);
// 5. Collision query between one object in group 1 and the entire group 2
manager2->collide(objects1[0], &collision_data, defaultCollisionFunction);
// 6. Distance query between one object in group 1 and the entire group 2
manager2->distance(objects1[0], &distance_data, defaultDistanceFunction); 
```


For more examples, please refer to the test folder:
- test_fcl_collision.cpp: provide examples for collision test
- test_fcl_distance.cpp: provide examples for distance test
- test_fcl_broadphase.cpp: provide examples for broadphase collision/distance test
- test_fcl_frontlist.cpp: provide examples for frontlist collision acceleration
#- test_fcl_global_penetration.cpp: provide examples for global penetration depth test
#- test_fcl_xmldata.cpp: provide examples for more global penetration depth test based on xml data
- test_fcl_octomap.cpp: provide examples for collision/distance computation between octomap data and other data types.
