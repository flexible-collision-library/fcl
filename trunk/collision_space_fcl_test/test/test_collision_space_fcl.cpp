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

/** \author E. Gil Jones */

#include <planning_models/kinematic_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <ros/package.h>

#include <collision_space/environmentODE.h>
#include <collision_space_fcl/environmentFCL.h>

#include <tf/transform_datatypes.h>
#include <arm_navigation_msgs/RobotState.h>

#include <geometric_shapes/shape_operations.h>

#define NUM_MESH_OBJECTS 10
#define NUM_RANDOM_OBJECTS 100
#define NUM_TEST_CONFIGURATIONS 1000

//urdf location relative to the planning_models path
static const std::string rel_path = "/test_urdf/robot.xml";

inline double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

inline geometry_msgs::Quaternion generateRandomUnitQuaternion() {

  geometry_msgs::Quaternion quat;
  quat.x = gen_rand(-1.0, 1.0);
  quat.w = gen_rand(-1.0, 1.0);
  quat.z = gen_rand(-1.0, 1.0);
  quat.w = gen_rand(-1.0, 1.0);

  double mag = sqrt(pow(quat.x, 2.0)+pow(quat.y, 2.0)+pow(quat.z, 2.0)+pow(quat.w, 2.0));

  quat.x /= mag;
  quat.y /= mag;
  quat.z /= mag;
  quat.w /= mag; 

  return quat;
}

inline std::string getNumberedString(const std::string &object, unsigned int num)
{
  std::stringstream my_string;
  my_string <<  object << num;
  return my_string.str();
}

class TestCollisionSpaceFCL : public testing::Test {
public:

  void spinThread() {
    lock_.lock();
    coll_space_->isCollision();
    lock_.unlock();
  }

protected:
  
  virtual void SetUp() {

    full_path_ = ros::package::getPath("planning_models")+rel_path;
    
    urdf_ok_ = urdf_model_.initFile(full_path_);

    std::vector<planning_models::KinematicModel::MultiDofConfig> multi_dof_configs;
    //now this should work with an identity transform
    planning_models::KinematicModel::MultiDofConfig config("base_joint");
    config.type = "Planar";
    config.parent_frame_id = "base_footprint";
    config.child_frame_id = "base_footprint";
    multi_dof_configs.push_back(config);

    std::vector<planning_models::KinematicModel::GroupConfig> gcs;
    planning_models::KinematicModel::GroupConfig left_arm("left_arm",
                                                          "torso_lift_link",
                                                          "l_wrist_roll_link");

    planning_models::KinematicModel::GroupConfig right_arm("right_arm",
                                                           "torso_lift_link",
                                                           "r_wrist_roll_link");
    gcs.push_back(left_arm);
    gcs.push_back(right_arm);
    kinematic_model_ = new planning_models::KinematicModel(urdf_model_,
                                                           gcs,
                                                           multi_dof_configs);
    coll_space_ode_ = new collision_space::EnvironmentModelODE();
    coll_space_ = new collision_space_fcl::EnvironmentModelFCL();

    workspace_x_extents_ = 1.0;
    workspace_y_extents_ = 1.0;
    workspace_z_extents_ = 1.0;
    ros::Time::init();
  };

  virtual void TearDown() {
    delete kinematic_model_;
    //    delete coll_space_;
    //    delete coll_space_ode_;
  }

  void getJointBoundsMap(const std::string &group, std::map<std::string, std::pair<double, double> > &joint_bounds_map)
  {
    const planning_models::KinematicModel::JointModelGroup* joint_model_group = kinematic_model_->getModelGroup(group);
    if(joint_model_group == NULL) {
      ROS_WARN_STREAM("No joint group " << group);
      return;
    }

    for(unsigned int i = 0; i < joint_model_group->getJointModels().size(); i++) {
      const planning_models::KinematicModel::JointModel* jm = joint_model_group->getJointModels()[i];
      std::pair<double, double> bounds;
      jm->getVariableBounds(jm->getName(), bounds);
      joint_bounds_map[jm->getName()] = bounds;
    }
  }

  void setupForRandomConfigurations()
  {
    srand(time(NULL));
    std::vector<std::string> links;
    kinematic_model_->getLinkModelNames(links);
    std::map<std::string, double> link_padding_map;
    
    collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm_ode(links, false);
    
    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  
    coll_space_ode_->setRobotModel(kinematic_model_, acm_ode, link_padding_map);  
    
    {
      //indented cause the state needs to cease to exist before we add the attached body
      planning_models::KinematicState state(kinematic_model_);
      state.setKinematicStateToDefault();
      
      coll_space_->updateRobotModel(&state);
      coll_space_ode_->updateRobotModel(&state);
    }
    
    ASSERT_FALSE(coll_space_->isEnvironmentCollision());
    ASSERT_FALSE(coll_space_ode_->isEnvironmentCollision());    
  }

  void testForRandomConfigurations()
  {
    getJointBoundsMap("right_arm",joint_bounds_map_right_);
    getJointBoundsMap("left_arm",joint_bounds_map_left_);

    std::vector<arm_navigation_msgs::RobotState> robot_states_right;
    std::vector<arm_navigation_msgs::RobotState> robot_states_left;
    // Now generate random robot states and test them for collisions
    for(unsigned int i=0; i < NUM_TEST_CONFIGURATIONS; i++)
    {
      arm_navigation_msgs::RobotState robot_state_right = generateRandomRobotStateWithinLimits(joint_bounds_map_right_);
      arm_navigation_msgs::RobotState robot_state_left = generateRandomRobotStateWithinLimits(joint_bounds_map_left_);
      robot_states_right.push_back(robot_state_right);
      robot_states_left.push_back(robot_state_left);
    }

    std::vector<bool> is_collision, is_collision_ode;
    is_collision.resize(robot_states_right.size());
    is_collision_ode.resize(robot_states_right.size());

    {
      //indented cause the state needs to cease to exist before we add the attached body
      planning_models::KinematicState state(kinematic_model_);
      state.setKinematicStateToDefault();
      planning_models::KinematicState::JointStateGroup *right_state = state.getJointStateGroup("right_arm");
      planning_models::KinematicState::JointStateGroup *left_state = state.getJointStateGroup("left_arm");

      ros::Time start_time = ros::Time::now();
      for(unsigned int i=0; i < robot_states_right.size(); i++)
      {
        right_state->setKinematicState(robot_states_right[i].joint_state.position);  
        left_state->setKinematicState(robot_states_left[i].joint_state.position);  
        coll_space_->updateRobotModel(&state);
        is_collision[i] = coll_space_->isEnvironmentCollision();
      }
      ros::Duration duration = ros::Time::now()-start_time;
      ROS_INFO("FCL: Collision time: %f",duration.toSec());

      start_time = ros::Time::now();
      for(unsigned int i=0; i < robot_states_right.size(); i++)
      {
        right_state->setKinematicState(robot_states_right[i].joint_state.position);  
        left_state->setKinematicState(robot_states_left[i].joint_state.position);  
        coll_space_ode_->updateRobotModel(&state);
        is_collision_ode[i] = coll_space_ode_->isEnvironmentCollision();
      }
      duration = ros::Time::now()-start_time;
      ROS_INFO("ODE: Collision time: %f",duration.toSec());

      unsigned int counter = 0;    
      for(unsigned int i=0; i < robot_states_right.size(); i++)
        if(is_collision_ode[i] != is_collision[i])
          counter++;
      ROS_DEBUG("%d mismatches between ODE and FCL result (out of %d)",(int) counter,(int) robot_states_right.size());

      ASSERT_TRUE(counter == 0);
    }
  }

  arm_navigation_msgs::RobotState generateRandomRobotStateWithinLimits(std::map<std::string, std::pair<double, double> > &joint_bounds_map) 
  {    
    arm_navigation_msgs::RobotState rs;
    rs.joint_state.header.frame_id = "torso_lift_link";
    rs.joint_state.header.stamp = ros::Time::now();

    for(std::map<std::string,std::pair<double, double> >::iterator it = joint_bounds_map.begin();
        it != joint_bounds_map.end();
        it++) {
      rs.joint_state.name.push_back(it->first);
      rs.joint_state.position.push_back(gen_rand(it->second.first, it->second.second));
    }
    return rs;
  }

  geometry_msgs::Pose generateRandomPoseInWorkspace() {
    
    geometry_msgs::Pose rp;
    rp.position.x = gen_rand(-workspace_x_extents_, workspace_x_extents_);
    rp.position.y = gen_rand(-workspace_y_extents_, workspace_y_extents_);
    rp.position.z = gen_rand(-workspace_z_extents_, workspace_z_extents_);

    rp.orientation = generateRandomUnitQuaternion();

    return rp;
    
  }


protected:

  boost::mutex lock_;
  urdf::Model urdf_model_;
  bool urdf_ok_;
  std::string full_path_;
  collision_space::EnvironmentModelODE* coll_space_ode_;
  collision_space_fcl::EnvironmentModelFCL* coll_space_;
  planning_models::KinematicModel* kinematic_model_;
  std::map<std::string, std::pair<double, double> > joint_bounds_map_right_, joint_bounds_map_left_;
  double workspace_x_extents_, workspace_y_extents_, workspace_z_extents_;
};



TEST_F(TestCollisionSpaceFCL, TestInit) {
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;

  {
    collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links,true);
    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
    //all AllowedCollisions set to true, so no collision
    ASSERT_FALSE(coll_space_->isCollision());
  }

  {
    collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links,false);

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
  
    //now we are in collision with nothing disabled
    ASSERT_TRUE(coll_space_->isCollision());
  }

  //one more time for good measure
  {
    collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links,false);

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
  
    //now we are in collision with nothing disabled
    ASSERT_TRUE(coll_space_->isCollision());
  }
}


TEST_F(TestCollisionSpaceFCL, TestACM) {
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  //first we get
  {
    collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);

    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    coll_space_->updateRobotModel(&state);

    //at default state in collision
    ASSERT_TRUE(coll_space_->isCollision());

    //now we get the full set of collisions in the default state
    std::vector<collision_space_fcl::EnvironmentModel::AllowedContact> ac;
    std::vector<collision_space_fcl::EnvironmentModel::Contact> contacts;
      
    coll_space_->getAllCollisionContacts(ac, contacts, 1);

    ASSERT_TRUE(contacts.size() > 1);
    //now we change all these pairwise to true
    for(unsigned int i = 0; i < contacts.size(); i++) {
      ASSERT_TRUE(contacts[i].body_type_1 == collision_space_fcl::EnvironmentModel::LINK);
      ASSERT_TRUE(contacts[i].body_type_2 == collision_space_fcl::EnvironmentModel::LINK);
      ASSERT_TRUE(acm.changeEntry(contacts[i].body_name_1,contacts[i].body_name_2, true));
    }

    coll_space_->setAlteredCollisionMatrix(acm);
    
    //with all of these disabled, no more collisions
    ASSERT_FALSE(coll_space_->isCollision());

    coll_space_->revertAlteredCollisionMatrix();
    ASSERT_TRUE(coll_space_->isCollision());
  }
}

TEST_F(TestCollisionSpaceFCL, TestAttachedObjects)
{
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
  coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);

    //now we get the full set of collisions in the default state
    std::vector<collision_space_fcl::EnvironmentModel::AllowedContact> ac;
    std::vector<collision_space_fcl::EnvironmentModel::Contact> contacts;
      
    coll_space_->getAllCollisionContacts(ac, contacts, 1);

    //now we change all these pairwise to true
    for(unsigned int i = 0; i < contacts.size(); i++) {
      ASSERT_TRUE(contacts[i].body_type_1 == collision_space_fcl::EnvironmentModel::LINK);
      ASSERT_TRUE(contacts[i].body_type_2 == collision_space_fcl::EnvironmentModel::LINK);
      ASSERT_TRUE(acm.changeEntry(contacts[i].body_name_1,contacts[i].body_name_2, true));
    }

    coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);
    coll_space_->updateRobotModel(&state);
  }
  
  //now we shouldn't be in collision
  ASSERT_FALSE(coll_space_->isCollision());  

  const planning_models::KinematicModel::LinkModel *link = kinematic_model_->getLinkModel("base_link");

  //first a single box
  shapes::Sphere* sphere1 = new shapes::Sphere();
  sphere1->radius = .1;

  shapes::Box* box2 = new shapes::Box();
  box2->size[0] = .05;
  box2->size[1] = .05;
  box2->size[2] = .05;
  
  std::vector<shapes::Shape*> shape_vector;
  shape_vector.push_back(sphere1);

  btTransform pose;
  pose.setIdentity();

  std::vector<btTransform> poses;
  poses.push_back(pose);
  
  std::vector<std::string> touch_links;

  planning_models::KinematicModel::AttachedBodyModel* ab1 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "box_1",
                                                           poses,
                                                           touch_links,
                                                           shape_vector);

  kinematic_model_->addAttachedBodyModel(link->getName(), ab1);
  coll_space_->updateAttachedBodies();

  const collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix& aft_attached
    = coll_space_->getDefaultAllowedCollisionMatrix();
  
  ASSERT_TRUE(aft_attached.hasEntry("box_1"));
  bool allowed;
  EXPECT_TRUE(aft_attached.getAllowedCollision("box_1", link->getName(), allowed));
  EXPECT_FALSE(allowed);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  //now it collides
  ASSERT_TRUE(coll_space_->isCollision());  

  kinematic_model_->clearLinkAttachedBodyModel(link->getName(), "box_1");
  coll_space_->updateAttachedBodies();

  ASSERT_FALSE(aft_attached.hasEntry("box_1"));

  //now adding an attached object with two boxes, this time with two objects
  shape_vector.clear();
  shape_vector.push_back(box2);
  pose.getOrigin().setX(.1);
  poses.clear();
  poses.push_back(pose);
  touch_links.push_back("r_gripper_palm_link");
  touch_links.push_back("r_gripper_r_finger_link");
  touch_links.push_back("r_gripper_l_finger_link");
  touch_links.push_back("r_gripper_r_finger_tip_link");
  touch_links.push_back("r_gripper_l_finger_tip_link");
  touch_links.push_back("base_link");
  
  planning_models::KinematicModel::AttachedBodyModel* ab2 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "box_2",
                                                           poses,
                                                           touch_links,
                                                           shape_vector);
  kinematic_model_->addAttachedBodyModel(link->getName(), ab2);
  coll_space_->updateAttachedBodies();

  ASSERT_TRUE(aft_attached.hasEntry("box_2"));
  EXPECT_TRUE(aft_attached.getAllowedCollision("box_2", link->getName(), allowed));
  EXPECT_TRUE(allowed);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  //now it doesn't collide
  ASSERT_FALSE(coll_space_->isCollision());  
}

TEST_F(TestCollisionSpaceFCL, TestStaticObjects)
{
  std::vector<std::string> links;
  kinematic_model_->getLinkModelNames(links);
  std::map<std::string, double> link_padding_map;
  
  collision_space_fcl::EnvironmentModel::AllowedCollisionMatrix acm(links, false);
  coll_space_->setRobotModel(kinematic_model_, acm, link_padding_map);  
  
  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  ASSERT_FALSE(coll_space_->isEnvironmentCollision());

  shapes::Sphere* sphere1 = new shapes::Sphere();
  sphere1->radius = .2;

  btTransform pose;
  pose.setIdentity();

  std::vector<btTransform> poses;
  poses.push_back(pose);

  std::vector<shapes::Shape*> shape_vector;
  shape_vector.push_back(sphere1);

  coll_space_->addObjects("obj1", shape_vector, poses);


  ASSERT_TRUE(coll_space_->isEnvironmentCollision());



  //Now test interactions between static and attached objects

  const planning_models::KinematicModel::LinkModel *link = kinematic_model_->getLinkModel("base_link");

  shapes::Box* att_box = new shapes::Box();
  att_box->size[0] = .05;
  att_box->size[1] = .05;
  att_box->size[2] = .05;
  
  std::vector<shapes::Shape*> att_shapes;
  att_shapes.push_back(att_box);

  btTransform att_pose;
  att_pose.setIdentity();

  std::vector<btTransform> att_poses;
  att_poses.push_back(att_pose);
  
  std::vector<std::string> touch_links;
  touch_links.push_back("base_link");
  touch_links.push_back("base_footprint");

  planning_models::KinematicModel::AttachedBodyModel* ab1 = 
    new planning_models::KinematicModel::AttachedBodyModel(link, "att1",
                                                           att_poses,
                                                           touch_links,
                                                           att_shapes);

  kinematic_model_->addAttachedBodyModel(link->getName(), ab1);
  coll_space_->updateAttachedBodies();

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  ASSERT_TRUE(coll_space_->isEnvironmentCollision());

  //now we get the full set of collisions in the default state
  std::vector<collision_space_fcl::EnvironmentModel::AllowedContact> ac;
  std::vector<collision_space_fcl::EnvironmentModel::Contact> contacts;
  
  coll_space_->getAllCollisionContacts(ac, contacts, 1);

  //now we change all these pairwise to true
  for(unsigned int i = 0; i < contacts.size(); i++) {
    if(contacts[i].body_type_1 == collision_space_fcl::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_1 == "obj1");
    }
    if(contacts[i].body_type_2 == collision_space_fcl::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_2 == "obj1");
    }
  }

  acm = coll_space_->getDefaultAllowedCollisionMatrix();
  bool allowed;
  ASSERT_TRUE(acm.getAllowedCollision("obj1","att1",allowed));
  EXPECT_FALSE(allowed);
  
  ASSERT_TRUE(acm.changeEntry(link->getName(), "obj1", true));
  ASSERT_TRUE(acm.changeEntry("base_footprint", "obj1", true));
  coll_space_->setAlteredCollisionMatrix(acm);

  {
    //indented cause the state needs to cease to exist before we add the attached body
    planning_models::KinematicState state(kinematic_model_);
    state.setKinematicStateToDefault();
    
    coll_space_->updateRobotModel(&state);
  }

  /* Remove because sphere-box collision does not work for mesh */
  // EXPECT_TRUE(coll_space_->isEnvironmentCollision());
  
  ASSERT_TRUE(acm.changeEntry("att1", "obj1", true));
  coll_space_->setAlteredCollisionMatrix(acm);
  
  allowed = false;
  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("att1","obj1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","att1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("base_link","obj1", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","base_link", allowed));
  EXPECT_TRUE(allowed);

  ASSERT_TRUE(coll_space_->getCurrentAllowedCollisionMatrix().getAllowedCollision("obj1","base_footprint", allowed));
  EXPECT_TRUE(allowed);
  
  EXPECT_FALSE(coll_space_->isEnvironmentCollision());
  contacts.clear();

  coll_space_->getAllCollisionContacts(ac, contacts, 1);

  //now we change all these pairwise to true
  for(unsigned int i = 0; i < contacts.size(); i++) {
    if(contacts[i].body_type_1 == collision_space_fcl::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_1 == "obj1");
      ROS_INFO_STREAM(contacts[i].body_name_2);
    }
    if(contacts[i].body_type_2 == collision_space_fcl::EnvironmentModel::OBJECT) {
      ASSERT_TRUE(contacts[i].body_name_2 == "obj1");
      ROS_INFO_STREAM(contacts[i].body_name_1);
    }
  }
}

TEST_F(TestCollisionSpaceFCL, TestODEvsFCLSphere)
{
  setupForRandomConfigurations();
  for(unsigned int i=0; i < NUM_RANDOM_OBJECTS; i++)
  {
    btTransform pose;
    shapes::Sphere* sphere1 = new shapes::Sphere();
    sphere1->radius = .1;    

    geometry_msgs::Pose pose_msg = generateRandomPoseInWorkspace();
    tf::poseMsgToTF(pose_msg,pose);

    std::vector<btTransform> poses;
    poses.push_back(pose);    

    std::vector<shapes::Shape*> shape_vector;
    shape_vector.push_back(sphere1);

    std::string object_id = getNumberedString("obj",i);

    coll_space_->addObjects(object_id, shape_vector, poses);
    coll_space_ode_->addObjects(object_id, shape_vector, poses);
  }
  testForRandomConfigurations();
}

TEST_F(TestCollisionSpaceFCL, TestODEvsFCLCylinder)
{
  setupForRandomConfigurations();
  for(unsigned int i=0; i < NUM_RANDOM_OBJECTS; i++)
  {
    btTransform pose;
    shapes::Cylinder* c1 = new shapes::Cylinder();
    c1->radius = .1;    
    c1->length = 0.3;

    geometry_msgs::Pose pose_msg = generateRandomPoseInWorkspace();
    tf::poseMsgToTF(pose_msg,pose);

    std::vector<btTransform> poses;
    poses.push_back(pose);    

    std::vector<shapes::Shape*> shape_vector;
    shape_vector.push_back(c1);

    std::string object_id = getNumberedString("obj",i);

    coll_space_->addObjects(object_id, shape_vector, poses);
    coll_space_ode_->addObjects(object_id, shape_vector, poses);
  }
  testForRandomConfigurations();
}

TEST_F(TestCollisionSpaceFCL, TestODEvsFCLBox)
{
  setupForRandomConfigurations();
  for(unsigned int i=0; i < NUM_RANDOM_OBJECTS; i++)
  {
    btTransform pose;
    shapes::Box* box = new shapes::Box();
    box->size[0] = 0.1;    
    box->size[1] = 0.2;
    box->size[2] = 0.3;

    geometry_msgs::Pose pose_msg = generateRandomPoseInWorkspace();
    tf::poseMsgToTF(pose_msg,pose);

    std::vector<btTransform> poses;
    poses.push_back(pose);    

    std::vector<shapes::Shape*> shape_vector;
    shape_vector.push_back(box);

    std::string object_id = getNumberedString("obj",i);

    coll_space_->addObjects(object_id, shape_vector, poses);
    coll_space_ode_->addObjects(object_id, shape_vector, poses);
  }
  testForRandomConfigurations();
}

TEST_F(TestCollisionSpaceFCL, TestODEvsFCLMesh)
{
  setupForRandomConfigurations();
  std::string full_path = ros::package::getPath("collision_space_fcl_test")+"/objects/meshes/9300.stl";
  for(unsigned int i=0; i < NUM_MESH_OBJECTS; i++)
  {
    shapes::Mesh* mesh = shapes::createMeshFromBinaryStl(full_path.c_str());
    ASSERT_TRUE(mesh);
    btTransform pose;
    geometry_msgs::Pose pose_msg = generateRandomPoseInWorkspace();
    tf::poseMsgToTF(pose_msg,pose);

    std::vector<btTransform> poses;
    poses.push_back(pose);    

    std::vector<shapes::Shape*> shape_vector;
    shape_vector.push_back(mesh);

    std::string object_id = getNumberedString("obj",i);

    coll_space_->addObjects(object_id, shape_vector, poses);
    coll_space_ode_->addObjects(object_id, shape_vector, poses);
    delete mesh;
  }
  testForRandomConfigurations();
  //  coll_space_->clearObjects();
  //  coll_space_ode_->clearObjects();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

