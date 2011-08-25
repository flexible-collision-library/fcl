/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

/** \author Ioan Sucan */

#ifndef COLLISION_SPACE_FCL_ENVIRONMENT_MODEL_
#define COLLISION_SPACE_FCL_ENVIRONMENT_MODEL_

#include "collision_space_fcl/environment_objects.h"
#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <geometric_shapes/bodies.h>
#include <LinearMath/btVector3.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


/** \brief Main namespace */
namespace collision_space_fcl
{
    
/** \brief A class describing an environment for a kinematic
    robot. This is the base (abstract) definition. Different
    implementations are possible. The class is aware of a set of
    obstacles and a robot model. The obstacles are placed in different
    namespaces so they can be added and removed selectively.
*/
class EnvironmentModel
{
public:
	
  enum BodyType {
    LINK,
    ATTACHED,
    OBJECT
  };

  /** \brief Definition of a contact point */
  struct Contact
  {
    /** \brief contact position */
    btVector3 pos;     
    /** \brief normal unit vector at contact */
    btVector3 normal;  
    /** \brief depth (penetration between bodies) */
    double depth;

    /** \brief The first body involved in the contact */
    std::string body_name_1;
    BodyType body_type_1;

    /** \brief The first body involved in the contact */
    std::string body_name_2;
    BodyType body_type_2;
  };

  /** \brief Definition of a contact that is allowed */
  struct AllowedContact
  {
    /// the bound where the contact is allowed 
    boost::shared_ptr<bodies::Body> bound;
	    
    /// the set of link names that are allowed to make contact
    std::vector<std::string> links;

    /// tha maximum depth for the contact
    double depth;
  };

  /** \brief Definition of a structure for the allowed collision matrix */
  /* False means that no collisions are allowed, true means ok */
  class AllowedCollisionMatrix
  {
  public:
    
    AllowedCollisionMatrix(){
      valid_ = true;
    }

    AllowedCollisionMatrix(const std::vector<std::string>& names,
                           bool allowed = false);

    AllowedCollisionMatrix(const std::vector<std::vector<bool> >& all_coll_vectors,
                           const std::map<std::string, unsigned int>& all_coll_indices);
    
    AllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

    bool getAllowedCollision(const std::string& name1, const std::string& name2,
                             bool& allowed_collision) const;

    bool getAllowedCollision(unsigned int i, unsigned int j,
                             bool& allowed_collision) const;

    bool hasEntry(const std::string& name) const;

    bool getEntryIndex(const std::string& name,
                               unsigned int& index) const;
    
    bool getEntryName(const unsigned int ind,
                      std::string& name) const;

    bool removeEntry(const std::string& name);

    bool addEntry(const std::string& name, bool allowed);

    bool changeEntry(bool allowed);
    
    bool changeEntry(const std::string& name, bool allowed);

    bool changeEntry(const std::string& name1,
                     const std::string& name2,
                     bool allowed);

    bool changeEntry(const unsigned int ind_1,
                     const unsigned int ind_2,
                     bool allowed);

    bool changeEntry(const std::string& name, 
                     const std::vector<std::string>& change_names,
                     bool allowed);

    bool changeEntry(const std::vector<std::string>& change_names_1,
                     const std::vector<std::string>& change_names_2,
                     bool allowed);

    bool getValid() const {
      return valid_;
    };

    unsigned int getSize() const {
      return allowed_entries_.size();
    }

    typedef boost::bimap<std::string, unsigned int> entry_type;

    const entry_type& getEntriesBimap() const {
      return allowed_entries_bimap_;
    }
    
  private:

    bool valid_;
    std::vector<std::vector<bool> > allowed_entries_;

    entry_type allowed_entries_bimap_;
  };

  EnvironmentModel(void)
  {
    verbose_ = false;
    objects_ = new EnvironmentObjects();
    use_altered_collision_matrix_ = false;
  }
	
  virtual ~EnvironmentModel(void)
  {
    if (objects_)
      delete objects_;
  }

  /**********************************************************************/
  /* Collision Environment Configuration                                */
  /**********************************************************************/
	
  /** \brief Add a robot model. Ignore robot links if their name is not
      specified in the string vector. The scale argument can be
      used to increase or decrease the size of the robot's
      bodies (multiplicative factor). The padding can be used to
      increase or decrease the robot's bodies with by an
      additive term */
  virtual void setRobotModel(const planning_models::KinematicModel* model, 
                             const AllowedCollisionMatrix& acm,
                             const std::map<std::string, double>& link_padding_map,
                             double default_padding = 0.0,
                             double scale = 1.0);

  /** \brief Get robot scale */
  double getRobotScale(void) const;
	
  /** \brief Get robot padding */
  double getRobotPadding(void) const;
	
  /** \brief Update the positions of the geometry used in collision detection */
  virtual void updateRobotModel(const planning_models::KinematicState* state) = 0;

  /** \brief Update the set of bodies that are attached to the robot (re-creates them) */
  virtual void updateAttachedBodies() = 0;
		
  /** \brief Get the robot model */
  const planning_models::KinematicModel* getRobotModel(void) const;
	
  /**********************************************************************/
  /* Collision Checking Routines                                        */
  /**********************************************************************/
	

  /** \brief Check if a model is in collision with environment and self. Contacts are not computed */
  virtual bool isCollision(void) const = 0;
	
  /** \brief Check for self collision. Contacts are not computed */
  virtual bool isSelfCollision(void) const = 0;

  /** \brief Check whether the model is in collision with the environment.  Self collisions are not checked */
  virtual bool isEnvironmentCollision(void) const = 0;
	
  /** \brief Get the list of contacts (collisions). The maximum number of contacts to be returned can be specified. If the value is 0, all found contacts are returned. */
  virtual bool getCollisionContacts(const std::vector<AllowedContact> &allowedContacts, std::vector<Contact> &contacts, unsigned int max_count = 1) const = 0;

  /** \brief This function will get the complete list of contacts between any two potentially colliding bodies.  The num per contacts specifies the number of contacts per pair that will be returned */
  virtual bool getAllCollisionContacts(const std::vector<AllowedContact> &allowedContacts, std::vector<Contact> &contacts, unsigned int num_per_contact = 1) const = 0;

  bool getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count = 1) const;
	
  /**********************************************************************/
  /* Collision Bodies                                                   */
  /**********************************************************************/
	
  /** \brief Remove all objects from collision model */
  virtual void clearObjects(void) = 0;
	
  /** \brief Remove objects from a specific namespace in the collision model */
  virtual void clearObjects(const std::string &ns) = 0;

  /** \brief Tells whether or not there is an object with the given name in the collision model */
  virtual bool hasObject(const std::string& ns) const = 0;
	
  /** \brief Add a static collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment. */
  virtual void addObject(const std::string &ns, shapes::StaticShape *shape) = 0;

  /** \brief Add a collision object to the map. The user releases ownership of the passed object. Memory allocated for the shape is freed by the collision environment.*/
  virtual void addObject(const std::string &ns, shapes::Shape* shape, const btTransform &pose) = 0;

  /** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. Memory allocated for the shapes is freed by the collision environment.*/
  virtual void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses) = 0;

  virtual void getAttachedBodyPoses(std::map<std::string, std::vector<btTransform> >& pose_map) const = 0;

  /** \briefs Sets a temporary robot padding on the indicated links */
  virtual void setAlteredLinkPadding(const std::map<std::string, double>& link_padding_map);

  /** \briefs Reverts link padding to that set at robot initialization */
  virtual void revertAlteredLinkPadding();

  const std::map<std::string,double>& getDefaultLinkPaddingMap() const;

  std::map<std::string,double> getCurrentLinkPaddingMap() const;

  double getCurrentLinkPadding(std::string name) const;
		
  /** \brief Get the objects currently contained in the model */
  const EnvironmentObjects* getObjects(void) const;
  
  const AllowedCollisionMatrix& getDefaultAllowedCollisionMatrix() const;
  
  const AllowedCollisionMatrix& getCurrentAllowedCollisionMatrix() const;

  /** \brief set the matrix for collision touch to use in lieu of the default settings */
  virtual void setAlteredCollisionMatrix(const AllowedCollisionMatrix& acm);

  /** \brief reverts to using default settings for allowed collisions */  
  virtual void revertAlteredCollisionMatrix();

  /**********************************************************************/
  /* Miscellaneous Routines                                             */
  /**********************************************************************/

  /** \brief Provide interface to a lock. Use carefully! */
  void lock(void) const;
	
  /** \brief Provide interface to a lock. Use carefully! */
  void unlock(void) const;

  /** \brief Enable/disable verbosity */
  void setVerbose(bool verbose);
	
  /** \brief Check the state of verbosity */
  bool getVerbose(void) const;
	
  /** \brief Clone the environment. */
  virtual EnvironmentModel* clone(void) const = 0;
	
protected:
        
  /** \brief Mutex used to lock the datastructure */
  mutable boost::recursive_mutex lock_;

  /** \brief Flag to indicate whether verbose mode is on */
  bool verbose_;

  /** \brief Loaded robot model */	
  const planning_models::KinematicModel* robot_model_;

  /** \brief List of objects contained in the environment */
  EnvironmentObjects *objects_;
	
  /** \brief Scaling used for robot links */
  double robot_scale_;

  /** \brief padding used for robot links */
  double default_robot_padding_;	

  AllowedCollisionMatrix default_collision_matrix_;
  AllowedCollisionMatrix altered_collision_matrix_;

  bool use_altered_collision_matrix_;

  std::map<std::string, double> default_link_padding_map_;
  std::map<std::string, double> altered_link_padding_map_;

  bool use_altered_link_padding_map_;
	
};
}

#endif

