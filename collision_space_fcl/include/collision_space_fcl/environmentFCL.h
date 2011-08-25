/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef COLLISION_SPACE_FCL_ENVIRONMENT_MODEL_FCL_H
#define COLLISION_SPACE_FCL_ENVIRONMENT_MODEL_FCL_H

#include "collision_space_fcl/environment.h"
#include "fcl/broad_phase_collision.h"
#include <map>
#include <vector>
#include <set>

namespace collision_space_fcl
{
    	
/** \brief A class describing an environment for a kinematic robot using ODE */
class EnvironmentModelFCL : public EnvironmentModel
{     

  friend bool collisionCallBack(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata);

public:
		
  EnvironmentModelFCL(void);

  virtual ~EnvironmentModelFCL(void);


  /** \brief Get the list of contacts (collisions) */
  virtual bool getCollisionContacts(const std::vector<AllowedContact>& allowedContacts, std::vector<Contact>& contacts, unsigned int max_count = 1) const;

  /** \brief This function will get the complete list of contacts between any two potentially colliding bodies.  The num per contacts specifies the number of contacts per pair that will be returned */
  virtual bool getAllCollisionContacts(const std::vector<AllowedContact>& allowedContacts, std::vector<Contact>& contacts, unsigned int num_per_contact = 1) const;

  /** \brief Check if a model is in collision */
  virtual bool isCollision(void) const;

  /** \brief Check if a model is in self collision */
  virtual bool isSelfCollision(void) const;

  /** \brief Check if a model is in environment collision */
  virtual bool isEnvironmentCollision(void) const;
	
  /** \brief Remove all objects from collision model */
  virtual void clearObjects(void);
	
  /** \brief Remove objects from a specific namespace in the collision model */
  virtual void clearObjects(const std::string& ns);

  /** \brief Tells whether or not there is an object with the given name in the collision model */
  virtual bool hasObject(const std::string& ns) const;
		
  /** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string& ns, shapes::StaticShape* shape);

  /** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string& ns, shapes::Shape* shape, const btTransform& pose);

  /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment. */
  virtual void addObjects(const std::string& ns, const std::vector<shapes::Shape*>& shapes, const std::vector<btTransform>& poses);

  virtual void getAttachedBodyPoses(std::map<std::string, std::vector<btTransform> >& pose_map) const;

  /** \brief Add a robot model. Ignore robot links if their name is not
      specified in the string vector. The scale argument can be
      used to increase or decrease the size of the robot's
      bodies (multiplicative factor). The padding can be used to
      increase or decrease the robot's bodies with by an
      additive term */
  virtual void setRobotModel(const planning_models::KinematicModel* model, 
                             const AllowedCollisionMatrix& allowed_collision_matrix,
                             const std::map<std::string, double>& link_padding_map,
                             double default_padding = 0.0,
                             double scale = 1.0); 

  /** \brief Update the positions of the geometry used in collision detection */
  virtual void updateRobotModel(const planning_models::KinematicState* state);

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) */
  virtual void updateAttachedBodies(void);

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) using the indicated padding or default if non-specified */
  virtual void updateAttachedBodies(const std::map<std::string, double>& link_padding_map);

  /** \briefs Sets a temporary robot padding on the indicated links */
  virtual void setAlteredLinkPadding(const std::map<std::string, double>& link_padding_map);

  /** \briefs Reverts link padding to that set at robot initialization */
  virtual void revertAlteredLinkPadding();

  /** \brief Clone the environment */
  virtual EnvironmentModel* clone(void) const;

protected:

  /** \brief Geometry for attachment */
  struct AttGeom
  {
    ~AttGeom()
    {
      for(unsigned int i = 0; i < geom.size(); i++)
      {
        delete geom[i];
      }
      for(unsigned int i = 0; i < padded_geom.size(); i++)
      {
        delete padded_geom[i];
      }
    }

    std::vector<fcl::CollisionObject*> geom; // managed by self_geom_manager
    std::vector<fcl::CollisionObject*> padded_geom; // for collision with environment geometry
    const planning_models::KinematicModel::AttachedBodyModel* att;
    unsigned int index;
  };

  /** \brief Geometry for Link */
  struct LinkGeom
  {
    ~LinkGeom()
    {
      for(unsigned int i = 0; i < geom.size(); i++)
      {
        delete geom[i];
      }
      for(unsigned int i = 0; i < padded_geom.size(); i++)
      {
        delete padded_geom[i];
      }
      deleteAttachedBodies();
    }
    
    void deleteAttachedBodies()
    {
      for(unsigned int i = 0; i < att_bodies.size(); i++)
      {
        delete att_bodies[i];
      }
      att_bodies.clear();
    }

    std::vector<fcl::CollisionObject*> geom; // managed by self_geom_manager
    std::vector<fcl::CollisionObject*> padded_geom; // for collision with environment geometry
    std::vector<AttGeom*> att_bodies;
    const planning_models::KinematicModel::LinkModel* link;
    unsigned int index;
  };

  struct CollisionData
  {
    CollisionData(void)
    {
      done = false;
      collides = false;
      max_contacts = 1;
      contacts = NULL;
      allowed_collision_matrix = NULL;
      allowed = NULL;
      exhaustive = false;
    }

    //these are parameters
    unsigned int max_contacts;
    const AllowedCollisionMatrix* allowed_collision_matrix;
    const std::map<fcl::CollisionObject*, std::pair<std::string, BodyType> >* geom_lookup_map;
    const std::vector<AllowedContact>* allowed;
    bool exhaustive;

    //these are for return info
    bool done;
    bool collides;
    std::vector<EnvironmentModelFCL::Contact>* contacts;

    //for the last collision found
    std::string body_name_1;
    BodyType body_type_1;

    std::string body_name_2;
    BodyType body_type_2;
  };

	
  struct ModelInfo
  {
    std::vector<LinkGeom*> link_geom;
  };
	
  struct CollisionNamespace
  {
    CollisionNamespace(const std::string& nm) : name(nm)
    {
      env_geom_manager.reset(new fcl::SSaPCollisionManager());
    }

    virtual ~CollisionNamespace(void)
    {
      clear();
    }
	    
    void clear(void)
    {
      for(unsigned int i = 0; i < geoms.size(); ++i)
        delete geoms[i];
      geoms.clear();
      env_geom_manager->clear();
    }
	    
    std::string name;
    std::vector<fcl::CollisionObject*> geoms;
    boost::shared_ptr<fcl::BroadPhaseCollisionManager> env_geom_manager;
  };
	
	
  /** \brief Internal function for collision detection */
  void testCollision(CollisionData* data) const;

  /** \brief Internal function for collision detection */
  void testSelfCollision(CollisionData* data) const;

  /** \brief Internal function for collision detection */
  void testEnvironmentCollision(CollisionData* data) const;

  /** \brief Internal function for collision detection */
  void testObjectCollision(CollisionNamespace* cn, CollisionData* data) const;

  fcl::CollisionObject* copyGeom(fcl::CollisionObject* geom) const;

  void createRobotModel();

  fcl::CollisionObject* createGeom(const shapes::Shape* shape, double scale, double padding);

  fcl::CollisionObject* createGeom(const shapes::StaticShape* shape);

  void updateGeom(fcl::CollisionObject* geom, const btTransform& pose) const;

  void addAttachedBody(LinkGeom* lg, const planning_models::KinematicModel::AttachedBodyModel* attm,
                       double padd);

  std::map<std::string, bool> attached_bodies_in_collision_matrix_;

  void setAttachedBodiesLinkPadding();

  void revertAttachedBodiesLinkPadding();

  void freeMemory(void);	
	
  ModelInfo model_geom_;
  std::map<std::string, CollisionNamespace*> coll_namespaces_;

  std::map<fcl::CollisionObject*, std::pair<std::string, BodyType> > geom_lookup_map_;

  bool previous_set_robot_model_;

  boost::shared_ptr<fcl::BroadPhaseCollisionManager> self_geom_manager;

};
}

#endif
