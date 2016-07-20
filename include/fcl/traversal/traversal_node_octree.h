/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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

#ifndef FCL_TRAVERSAL_NODE_OCTREE_H
#define FCL_TRAVERSAL_NODE_OCTREE_H

#include "fcl/config.h"
#if not(FCL_HAVE_OCTOMAP)
#error "This header requires fcl to be compiled with octomap support"
#endif

#include "fcl/collision_data.h"
#include "fcl/traversal/traversal_node_base.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "fcl/octree.h"
#include "fcl/BVH/BVH_model.h"

namespace fcl
{

/// @brief Algorithms for collision related with octree
template<typename NarrowPhaseSolver>
class OcTreeSolver
{
private:
  const NarrowPhaseSolver* solver;

  mutable const CollisionRequest* crequest;
  mutable const DistanceRequest* drequest;

  mutable CollisionResult* cresult;
  mutable DistanceResult* dresult;

public:
  OcTreeSolver(const NarrowPhaseSolver* solver_) : solver(solver_),
                                                   crequest(NULL),
                                                   drequest(NULL),
                                                   cresult(NULL),
                                                   dresult(NULL)
  {
  }

  /// @brief collision between two octrees
  void OcTreeIntersect(const OcTree* tree1, const OcTree* tree2,
                       const Transform3f& tf1, const Transform3f& tf2,
                       const CollisionRequest& request_,
                       CollisionResult& result_) const
  {
    crequest = &request_;
    cresult = &result_;
    
    OcTreeIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), 
                           tree2, tree2->getRoot(), tree2->getRootBV(), 
                           tf1, tf2);
  }

  /// @brief distance between two octrees
  void OcTreeDistance(const OcTree* tree1, const OcTree* tree2,
                      const Transform3f& tf1, const Transform3f& tf2,
                      const DistanceRequest& request_,
                      DistanceResult& result_) const
  {
    drequest = &request_;
    dresult = &result_;

    OcTreeDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(), 
                          tree2, tree2->getRoot(), tree2->getRootBV(),
                          tf1, tf2);
  }

  /// @brief collision between octree and mesh
  template<typename BV>
  void OcTreeMeshIntersect(const OcTree* tree1, const BVHModel<BV>* tree2,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const
  {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                               tree2, 0,
                               tf1, tf2);
  }

  /// @brief distance between octree and mesh
  template<typename BV>
  void OcTreeMeshDistance(const OcTree* tree1, const BVHModel<BV>* tree2,
                          const Transform3f& tf1, const Transform3f& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const
  {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                              tree2, 0,
                              tf1, tf2);
  }

  /// @brief collision between mesh and octree
  template<typename BV>
  void MeshOcTreeIntersect(const BVHModel<BV>* tree1, const OcTree* tree2,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const CollisionRequest& request_,
                           CollisionResult& result_) const
  
  {
    crequest = &request_;
    cresult = &result_;

    OcTreeMeshIntersectRecurse(tree2, tree2->getRoot(), tree2->getRootBV(),
                               tree1, 0,
                               tf2, tf1);
  }

  /// @brief distance between mesh and octree
  template<typename BV>
  void MeshOcTreeDistance(const BVHModel<BV>* tree1, const OcTree* tree2,
                          const Transform3f& tf1, const Transform3f& tf2,
                          const DistanceRequest& request_,
                          DistanceResult& result_) const
  {
    drequest = &request_;
    dresult = &result_;

    OcTreeMeshDistanceRecurse(tree1, 0,
                              tree2, tree2->getRoot(), tree2->getRootBV(),
                              tf1, tf2);
  }

  /// @brief collision between octree and shape
  template<typename S>
  void OcTreeShapeIntersect(const OcTree* tree, const S& s,
                            const Transform3f& tf1, const Transform3f& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const
  {
    crequest = &request_;
    cresult = &result_;

    AABB bv2;
    computeBV<AABB>(s, Transform3f(), bv2);
    OBB obb2;
    convertBV(bv2, tf2, obb2);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(),
                                s, obb2,
                                tf1, tf2);
    
  }

  /// @brief collision between shape and octree
  template<typename S>
  void ShapeOcTreeIntersect(const S& s, const OcTree* tree,
                            const Transform3f& tf1, const Transform3f& tf2,
                            const CollisionRequest& request_,
                            CollisionResult& result_) const
  {
    crequest = &request_;
    cresult = &result_;

    AABB bv1;
    computeBV<AABB>(s, Transform3f(), bv1);
    OBB obb1;
    convertBV(bv1, tf1, obb1);
    OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(),
                                s, obb1,
                                tf2, tf1);
  }

  /// @brief distance between octree and shape
  template<typename S>
  void OcTreeShapeDistance(const OcTree* tree, const S& s,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const
  {
    drequest = &request_;
    dresult = &result_;

    AABB aabb2;
    computeBV<AABB>(s, tf2, aabb2);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(),
                               s, aabb2,
                               tf1, tf2);
  }

  /// @brief distance between shape and octree
  template<typename S>
  void ShapeOcTreeDistance(const S& s, const OcTree* tree,
                           const Transform3f& tf1, const Transform3f& tf2,
                           const DistanceRequest& request_,
                           DistanceResult& result_) const
  {
    drequest = &request_;
    dresult = &result_;

    AABB aabb1;
    computeBV<AABB>(s, tf1, aabb1);
    OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(),
                               s, aabb1,
                               tf2, tf1);
  }
  

private:
  template<typename S>
  bool OcTreeShapeDistanceRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                                  const S& s, const AABB& aabb2,
                                  const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!tree1->nodeHasChildren(root1))
    {
      if(tree1->isNodeOccupied(root1))
      {
        Box box;
        Transform3f box_tf;
        constructBox(bv1, tf1, box, box_tf);
 
        FCL_REAL dist;
        Vec3f closest_p1, closest_p2;
        solver->shapeDistance(box, box_tf, s, tf2, &dist, &closest_p1, &closest_p2);
        
        dresult->update(dist, tree1, &s, root1 - tree1->getRoot(), DistanceResult::NONE, closest_p1, closest_p2);
        
        return drequest->isSatisfied(*dresult);
      }
      else
        return false;
    }

    if(!tree1->isNodeOccupied(root1)) return false;
    
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);
        
        AABB aabb1;
        convertBV(child_bv, tf1, aabb1);
        FCL_REAL d = aabb1.distance(aabb2);
        if(d < dresult->min_distance)
        {
          if(OcTreeShapeDistanceRecurse(tree1, child, child_bv, s, aabb2, tf1, tf2))
            return true;
        }        
      }
    }

    return false;
  }

  template<typename S>
  bool OcTreeShapeIntersectRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                                   const S& s, const OBB& obb2,
                                   const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!root1)
    {
      OBB obb1;
      convertBV(bv1, tf1, obb1);
      if(obb1.overlap(obb2))
      {
        Box box;
        Transform3f box_tf;
        constructBox(bv1, tf1, box, box_tf);

        if(solver->shapeIntersect(box, box_tf, s, tf2, NULL))
        {
          AABB overlap_part;
          AABB aabb1, aabb2;
          computeBV<AABB, Box>(box, box_tf, aabb1);
          computeBV<AABB, S>(s, tf2, aabb2);
          aabb1.overlap(aabb2, overlap_part);
          cresult->addCostSource(CostSource(overlap_part, tree1->getOccupancyThres() * s.cost_density), crequest->num_max_cost_sources);
        }
      }

      return false;
    }
    else if(!tree1->nodeHasChildren(root1))
    {
      if(tree1->isNodeOccupied(root1) && s.isOccupied()) // occupied area
      {
        OBB obb1;
        convertBV(bv1, tf1, obb1);
        if(obb1.overlap(obb2))
        {
          Box box;
          Transform3f box_tf;
          constructBox(bv1, tf1, box, box_tf);

          bool is_intersect = false;
          if(!crequest->enable_contact)
          {
            if(solver->shapeIntersect(box, box_tf, s, tf2, NULL))
            {
              is_intersect = true;
              if(cresult->numContacts() < crequest->num_max_contacts)
                cresult->addContact(Contact(tree1, &s, root1 - tree1->getRoot(), Contact::NONE));
            }
          }
          else
          {
            std::vector<ContactPoint> contacts;
            if(solver->shapeIntersect(box, box_tf, s, tf2, &contacts))
            {
              is_intersect = true;
              if(crequest->num_max_contacts > cresult->numContacts())
              {
                const size_t free_space = crequest->num_max_contacts - cresult->numContacts();
                size_t num_adding_contacts;

                // If the free space is not enough to add all the new contacts, we add contacts in descent order of penetration depth.
                if (free_space < contacts.size())
                {
                  std::partial_sort(contacts.begin(), contacts.begin() + free_space, contacts.end(), std::bind(comparePenDepth, std::placeholders::_2, std::placeholders::_1));
                  num_adding_contacts = free_space;
                }
                else
                {
                  num_adding_contacts = contacts.size();
                }

                for(size_t i = 0; i < num_adding_contacts; ++i)
                  cresult->addContact(Contact(tree1, &s, root1 - tree1->getRoot(), Contact::NONE, contacts[i].pos, contacts[i].normal, contacts[i].penetration_depth));
              }
            }
          }

          if(is_intersect && crequest->enable_cost)
          {
            AABB overlap_part;
            AABB aabb1, aabb2;
            computeBV<AABB, Box>(box, box_tf, aabb1);
            computeBV<AABB, S>(s, tf2, aabb2);
            aabb1.overlap(aabb2, overlap_part);
          }

          return crequest->isSatisfied(*cresult);
        }
        else return false;
      }
      else if(!tree1->isNodeFree(root1) && !s.isFree() && crequest->enable_cost) // uncertain area
      {
        OBB obb1;
        convertBV(bv1, tf1, obb1);
        if(obb1.overlap(obb2))
        {
          Box box;
          Transform3f box_tf;
          constructBox(bv1, tf1, box, box_tf);

          if(solver->shapeIntersect(box, box_tf, s, tf2, NULL))
          {
            AABB overlap_part;
            AABB aabb1, aabb2;
            computeBV<AABB, Box>(box, box_tf, aabb1);
            computeBV<AABB, S>(s, tf2, aabb2);
            aabb1.overlap(aabb2, overlap_part);
          }
        }
        
        return false;
      }
      else // free area
        return false;
    }

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least of one the nodes is free; OR
    ///           2) (two uncertain nodes or one node occupied and one node uncertain) AND cost not required
    if(tree1->isNodeFree(root1) || s.isFree()) return false;
    else if((tree1->isNodeUncertain(root1) || s.isUncertain()) && !crequest->enable_cost) return false;
    else
    {
      OBB obb1;
      convertBV(bv1, tf1, obb1);
      if(!obb1.overlap(obb2)) return false;
    }

    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);
        
        if(OcTreeShapeIntersectRecurse(tree1, child, child_bv, s, obb2, tf1, tf2))
          return true;
      }
      else if(!s.isFree() && crequest->enable_cost)
      {
        AABB child_bv;
        computeChildBV(bv1, i, child_bv);

        if(OcTreeShapeIntersectRecurse(tree1, NULL, child_bv, s, obb2, tf1, tf2))
          return true;
      }
    }

    return false;    
  }

  template<typename BV>
  bool OcTreeMeshDistanceRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                                 const BVHModel<BV>* tree2, int root2,
                                 const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf())
    {
      if(tree1->isNodeOccupied(root1))
      {
        Box box;
        Transform3f box_tf;
        constructBox(bv1, tf1, box, box_tf);

        int primitive_id = tree2->getBV(root2).primitiveId();
        const Triangle& tri_id = tree2->tri_indices[primitive_id];
        const Vec3f& p1 = tree2->vertices[tri_id[0]];
        const Vec3f& p2 = tree2->vertices[tri_id[1]];
        const Vec3f& p3 = tree2->vertices[tri_id[2]];
        
        FCL_REAL dist;
        Vec3f closest_p1, closest_p2;
        solver->shapeTriangleDistance(box, box_tf, p1, p2, p3, tf2, &dist, &closest_p1, &closest_p2);

        dresult->update(dist, tree1, tree2, root1 - tree1->getRoot(), primitive_id);

        return drequest->isSatisfied(*dresult);
      }
      else
        return false;
    }

    if(!tree1->isNodeOccupied(root1)) return false;

    if(tree2->getBV(root2).isLeaf() || (tree1->nodeHasChildren(root1) && (bv1.size() > tree2->getBV(root2).bv.size())))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree1->nodeChildExists(root1, i))
        {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(child_bv, tf1, aabb1);
          convertBV(tree2->getBV(root2).bv, tf2, aabb2);
          d = aabb1.distance(aabb2);
          
          if(d < dresult->min_distance)
          {
            if(OcTreeMeshDistanceRecurse(tree1, child, child_bv, tree2, root2, tf1, tf2))
              return true;
          }
        }
      }
    }
    else
    {
      FCL_REAL d;
      AABB aabb1, aabb2;
      convertBV(bv1, tf1, aabb1);
      int child = tree2->getBV(root2).leftChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);

      if(d < dresult->min_distance)
      {
        if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1, tf2))
          return true;
      }

      child = tree2->getBV(root2).rightChild();
      convertBV(tree2->getBV(child).bv, tf2, aabb2);
      d = aabb1.distance(aabb2);
      
      if(d < dresult->min_distance)
      {
        if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, child, tf1, tf2))
          return true;      
      }
    }

    return false;
  }


  template<typename BV>
  bool OcTreeMeshIntersectRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                                  const BVHModel<BV>* tree2, int root2,
                                  const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!root1)
    {
      if(tree2->getBV(root2).isLeaf())
      {
        OBB obb1, obb2;
        convertBV(bv1, tf1, obb1);
        convertBV(tree2->getBV(root2).bv, tf2, obb2);
        if(obb1.overlap(obb2))
        {
          Box box;
          Transform3f box_tf;
          constructBox(bv1, tf1, box, box_tf);

          int primitive_id = tree2->getBV(root2).primitiveId();
          const Triangle& tri_id = tree2->tri_indices[primitive_id];
          const Vec3f& p1 = tree2->vertices[tri_id[0]];
          const Vec3f& p2 = tree2->vertices[tri_id[1]];
          const Vec3f& p3 = tree2->vertices[tri_id[2]];
        
          if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, NULL, NULL, NULL))
          {
            AABB overlap_part;
            AABB aabb1;
            computeBV<AABB, Box>(box, box_tf, aabb1);
            AABB aabb2(tf2.transform(p1), tf2.transform(p2), tf2.transform(p3));
            aabb1.overlap(aabb2, overlap_part);
            cresult->addCostSource(CostSource(overlap_part, tree1->getOccupancyThres() * tree2->cost_density), crequest->num_max_cost_sources);
          }
        }

        return false;
      }
      else
      {
        if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).leftChild(), tf1, tf2))
          return true;

        if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).rightChild(), tf1, tf2))
          return true;

        return false;
      }
    }
    else if(!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf())
    {
      if(tree1->isNodeOccupied(root1) && tree2->isOccupied())
      {
        OBB obb1, obb2;
        convertBV(bv1, tf1, obb1);
        convertBV(tree2->getBV(root2).bv, tf2, obb2);
        if(obb1.overlap(obb2))
        {
          Box box;
          Transform3f box_tf;
          constructBox(bv1, tf1, box, box_tf);

          int primitive_id = tree2->getBV(root2).primitiveId();
          const Triangle& tri_id = tree2->tri_indices[primitive_id];
          const Vec3f& p1 = tree2->vertices[tri_id[0]];
          const Vec3f& p2 = tree2->vertices[tri_id[1]];
          const Vec3f& p3 = tree2->vertices[tri_id[2]];
        
          bool is_intersect = false;
          if(!crequest->enable_contact)
          {
            if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, NULL, NULL, NULL))
            {
              is_intersect = true;
              if(cresult->numContacts() < crequest->num_max_contacts)
                cresult->addContact(Contact(tree1, tree2, root1 - tree1->getRoot(), primitive_id));
            }
          }
          else
          {
            Vec3f contact;
            FCL_REAL depth;
            Vec3f normal;

            if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, &contact, &depth, &normal))
            {
              is_intersect = true;
              if(cresult->numContacts() < crequest->num_max_contacts)
                cresult->addContact(Contact(tree1, tree2, root1 - tree1->getRoot(), primitive_id, contact, normal, depth));
            }
          }

          if(is_intersect && crequest->enable_cost)
          {
            AABB overlap_part;
            AABB aabb1;
            computeBV<AABB, Box>(box, box_tf, aabb1);
            AABB aabb2(tf2.transform(p1), tf2.transform(p2), tf2.transform(p3));
            aabb1.overlap(aabb2, overlap_part);
	    cresult->addCostSource(CostSource(overlap_part, root1->getOccupancy() * tree2->cost_density), crequest->num_max_cost_sources);
          }

          return crequest->isSatisfied(*cresult);
        }
        else
          return false;
      }
      else if(!tree1->isNodeFree(root1) && !tree2->isFree() && crequest->enable_cost) // uncertain area
      {
        OBB obb1, obb2;
        convertBV(bv1, tf1, obb1);
        convertBV(tree2->getBV(root2).bv, tf2, obb2);
        if(obb1.overlap(obb2))
        {
          Box box;
          Transform3f box_tf;
          constructBox(bv1, tf1, box, box_tf);

          int primitive_id = tree2->getBV(root2).primitiveId();
          const Triangle& tri_id = tree2->tri_indices[primitive_id];
          const Vec3f& p1 = tree2->vertices[tri_id[0]];
          const Vec3f& p2 = tree2->vertices[tri_id[1]];
          const Vec3f& p3 = tree2->vertices[tri_id[2]];
        
          if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, NULL, NULL, NULL))
          {
            AABB overlap_part;
            AABB aabb1;
            computeBV<AABB, Box>(box, box_tf, aabb1);
            AABB aabb2(tf2.transform(p1), tf2.transform(p2), tf2.transform(p3));
            aabb1.overlap(aabb2, overlap_part);
	    cresult->addCostSource(CostSource(overlap_part, root1->getOccupancy() * tree2->cost_density), crequest->num_max_cost_sources);
          }
        }
        
        return false;
      }
      else // free area
        return false;
    }

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node uncertain) AND cost not required
    if(tree1->isNodeFree(root1) || tree2->isFree()) return false;
    else if((tree1->isNodeUncertain(root1) || tree2->isUncertain()) && !crequest->enable_cost) return false;
    else
    {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(tree2->getBV(root2).bv, tf2, obb2);
      if(!obb1.overlap(obb2)) return false;      
    }
   
    if(tree2->getBV(root2).isLeaf() || (tree1->nodeHasChildren(root1) && (bv1.size() > tree2->getBV(root2).bv.size())))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree1->nodeChildExists(root1, i))
        {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);
          
          if(OcTreeMeshIntersectRecurse(tree1, child, child_bv, tree2, root2, tf1, tf2))
            return true;
        }
	else if(!tree2->isFree() && crequest->enable_cost)
        {
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          if(OcTreeMeshIntersectRecurse(tree1, NULL, child_bv, tree2, root2, tf1, tf2))
            return true;
        }
      }
    }
    else
    {
      if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).leftChild(), tf1, tf2))
        return true;

      if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).rightChild(), tf1, tf2))
        return true;      

    }

    return false;
  }

  bool OcTreeDistanceRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                             const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& bv2,
                             const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2))
    {
      if(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2))
      {
        Box box1, box2;
        Transform3f box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        FCL_REAL dist;
        Vec3f closest_p1, closest_p2;
        solver->shapeDistance(box1, box1_tf, box2, box2_tf, &dist, &closest_p1, &closest_p2);

        dresult->update(dist, tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot(), closest_p1, closest_p2);
        
        return drequest->isSatisfied(*dresult);
      }
      else
        return false;
    }

    if(!tree1->isNodeOccupied(root1) || !tree2->isNodeOccupied(root2)) return false;

    if(!tree2->nodeHasChildren(root2) || (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size())))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree1->nodeChildExists(root1, i))
        {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if(d < dresult->min_distance)
          {
          
            if(OcTreeDistanceRecurse(tree1, child, child_bv, tree2, root2, bv2, tf1, tf2))
              return true;
          }
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree2->nodeChildExists(root2, i))
        {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          FCL_REAL d;
          AABB aabb1, aabb2;
          convertBV(bv1, tf1, aabb1);
          convertBV(bv2, tf2, aabb2);
          d = aabb1.distance(aabb2);

          if(d < dresult->min_distance)
          {
            if(OcTreeDistanceRecurse(tree1, root1, bv1, tree2, child, child_bv, tf1, tf2))
              return true;
          }
        }
      }
    }
    
    return false;
  }


  bool OcTreeIntersectRecurse(const OcTree* tree1, const OcTree::OcTreeNode* root1, const AABB& bv1,
                              const OcTree* tree2, const OcTree::OcTreeNode* root2, const AABB& bv2,
                              const Transform3f& tf1, const Transform3f& tf2) const
  {
    if(!root1 && !root2)
    {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bv2, tf2, obb2);

      if(obb1.overlap(obb2))
      {
        Box box1, box2;
        Transform3f box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);
        
        AABB overlap_part;
        AABB aabb1, aabb2;
        computeBV<AABB, Box>(box1, box1_tf, aabb1);
        computeBV<AABB, Box>(box2, box2_tf, aabb2);
        aabb1.overlap(aabb2, overlap_part);
        cresult->addCostSource(CostSource(overlap_part, tree1->getOccupancyThres() * tree2->getOccupancyThres()), crequest->num_max_cost_sources);
      }

      return false;
    }
    else if(!root1 && root2)
    {
      if(tree2->nodeHasChildren(root2))
      {
        for(unsigned int i = 0; i < 8; ++i)
        {
          if(tree2->nodeChildExists(root2, i))
          {
            const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
            AABB child_bv;
            computeChildBV(bv2, i, child_bv);
            if(OcTreeIntersectRecurse(tree1, NULL, bv1, tree2, child, child_bv, tf1, tf2))
              return true;
          }
          else 
          {
            AABB child_bv;
            computeChildBV(bv2, i, child_bv);
            if(OcTreeIntersectRecurse(tree1, NULL, bv1, tree2, NULL, child_bv, tf1, tf2))
              return true;
          }
        }
      }
      else
      {
        if(OcTreeIntersectRecurse(tree1, NULL, bv1, tree2, NULL, bv2, tf1, tf2))
          return true;
      }
      
      return false;
    }
    else if(root1 && !root2)
    {
      if(tree1->nodeHasChildren(root1))
      {
        for(unsigned int i = 0; i < 8; ++i)
        {
          if(tree1->nodeChildExists(root1, i))
          {
            const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
            AABB child_bv;
            computeChildBV(bv1, i,  child_bv);
            if(OcTreeIntersectRecurse(tree1, child, child_bv, tree2, NULL, bv2, tf1, tf2))
              return true;
          }
          else
          {
            AABB child_bv;
            computeChildBV(bv1, i, child_bv);
            if(OcTreeIntersectRecurse(tree1, NULL, child_bv, tree2, NULL, bv2, tf1, tf2))
              return true;
          }
        }
      }
      else
      {
        if(OcTreeIntersectRecurse(tree1, NULL, bv1, tree2, NULL, bv2, tf1, tf2))
          return true;
      }
      
      return false;
    }
    else if(!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2))
    {
      if(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2)) // occupied area
      {
        bool is_intersect = false;
        if(!crequest->enable_contact)
        {
          OBB obb1, obb2;
          convertBV(bv1, tf1, obb1);
          convertBV(bv2, tf2, obb2);
          
          if(obb1.overlap(obb2))
          {
            is_intersect = true;
            if(cresult->numContacts() < crequest->num_max_contacts)
              cresult->addContact(Contact(tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot()));
          }
        }
        else
        {
          Box box1, box2;
          Transform3f box1_tf, box2_tf;
          constructBox(bv1, tf1, box1, box1_tf);
          constructBox(bv2, tf2, box2, box2_tf);

          std::vector<ContactPoint> contacts;
          if(solver->shapeIntersect(box1, box1_tf, box2, box2_tf, &contacts))
          {
            is_intersect = true;
            if(crequest->num_max_contacts > cresult->numContacts())
            {
              const size_t free_space = crequest->num_max_contacts - cresult->numContacts();
              size_t num_adding_contacts;

              // If the free space is not enough to add all the new contacts, we add contacts in descent order of penetration depth.
              if (free_space < contacts.size())
              {
                std::partial_sort(contacts.begin(), contacts.begin() + free_space, contacts.end(), std::bind(comparePenDepth, std::placeholders::_2, std::placeholders::_1));
                num_adding_contacts = free_space;
              }
              else
              {
                num_adding_contacts = contacts.size();
              }

              for(size_t i = 0; i < num_adding_contacts; ++i)
                cresult->addContact(Contact(tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot(), contacts[i].pos, contacts[i].normal, contacts[i].penetration_depth));
            }
          }
        }

        if(is_intersect && crequest->enable_cost)
        {
          Box box1, box2;
          Transform3f box1_tf, box2_tf;
          constructBox(bv1, tf1, box1, box1_tf);
          constructBox(bv2, tf2, box2, box2_tf);

          AABB overlap_part;
          AABB aabb1, aabb2;
          computeBV<AABB, Box>(box1, box1_tf, aabb1);
          computeBV<AABB, Box>(box2, box2_tf, aabb2);
          aabb1.overlap(aabb2, overlap_part);
	  cresult->addCostSource(CostSource(overlap_part, root1->getOccupancy() * root2->getOccupancy()), crequest->num_max_cost_sources);
        }

        return crequest->isSatisfied(*cresult);
      }
      else if(!tree1->isNodeFree(root1) && !tree2->isNodeFree(root2) && crequest->enable_cost) // uncertain area (here means both are uncertain or one uncertain and one occupied)
      {
        OBB obb1, obb2;
        convertBV(bv1, tf1, obb1);
        convertBV(bv2, tf2, obb2);
        
        if(obb1.overlap(obb2))
        {
          Box box1, box2;
          Transform3f box1_tf, box2_tf;
          constructBox(bv1, tf1, box1, box1_tf);
          constructBox(bv2, tf2, box2, box2_tf);

          AABB overlap_part;
          AABB aabb1, aabb2;
          computeBV<AABB, Box>(box1, box1_tf, aabb1);
          computeBV<AABB, Box>(box2, box2_tf, aabb2);
          aabb1.overlap(aabb2, overlap_part);
	  cresult->addCostSource(CostSource(overlap_part, root1->getOccupancy() * root2->getOccupancy()), crequest->num_max_cost_sources);
        }

        return false;
      }
      else // free area (at least one node is free)
        return false;
    }

    /// stop when 1) bounding boxes of two objects not overlap; OR
    ///           2) at least one of the nodes is free; OR
    ///           2) (two uncertain nodes OR one node occupied and one node uncertain) AND cost not required
    if(tree1->isNodeFree(root1) || tree2->isNodeFree(root2)) return false;
    else if((tree1->isNodeUncertain(root1) || tree2->isNodeUncertain(root2)) && !crequest->enable_cost) return false;
    else 
    {
      OBB obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bv2, tf2, obb2);
      if(!obb1.overlap(obb2)) return false;
    }

    if(!tree2->nodeHasChildren(root2) || (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size())))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree1->nodeChildExists(root1, i))
        {
          const OcTree::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);
        
          if(OcTreeIntersectRecurse(tree1, child, child_bv, 
                                    tree2, root2, bv2,
                                    tf1, tf2))
            return true;
        }
        else if(!tree2->isNodeFree(root2) && crequest->enable_cost)
        {
          AABB child_bv;
          computeChildBV(bv1, i, child_bv);
          
          if(OcTreeIntersectRecurse(tree1, NULL, child_bv,
                                    tree2, root2, bv2,
                                    tf1, tf2))
            return true;
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree2->nodeChildExists(root2, i))
        {
          const OcTree::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);
          
          if(OcTreeIntersectRecurse(tree1, root1, bv1,
                                    tree2, child, child_bv,
                                    tf1, tf2))
            return true;
        }
        else if(!tree1->isNodeFree(root1) && crequest->enable_cost)
        {
          AABB child_bv;
          computeChildBV(bv2, i, child_bv);

          if(OcTreeIntersectRecurse(tree1, root1, bv1,
                                    tree2, NULL, child_bv,
                                    tf1, tf2))
            return true;
        }
      }
    }

    return false;
  }
};




/// @brief Traversal node for octree collision
template<typename NarrowPhaseSolver>
class OcTreeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  OcTreeCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVTesting(int, int) const
  {
    return false;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeIntersect(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for octree distance
template<typename NarrowPhaseSolver>
class OcTreeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  OcTreeDistanceTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }


  FCL_REAL BVTesting(int, int) const
  {
    return -1;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const OcTree* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for shape-octree collision
template<typename S, typename NarrowPhaseSolver>
class ShapeOcTreeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  ShapeOcTreeCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVTesting(int, int) const
  {
    return false;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeShapeIntersect(model2, *model1, tf2, tf1, request, *result);
  }

  const S* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for octree-shape collision
template<typename S, typename NarrowPhaseSolver>
class OcTreeShapeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  OcTreeShapeCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVTesting(int, int) const
  {
    return false;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeShapeIntersect(model1, *model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const S* model2;

  Transform3f tf1, tf2;
 
  const OcTreeSolver<NarrowPhaseSolver>* otsolver;  
};

/// @brief Traversal node for shape-octree distance
template<typename S, typename NarrowPhaseSolver>
class ShapeOcTreeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  ShapeOcTreeDistanceTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;
    
    otsolver = NULL;
  }

  FCL_REAL BVTesting(int, int) const
  {
    return -1;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeShapeDistance(model2, *model1, tf2, tf1, request, *result);
  }

  const S* model1;
  const OcTree* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for octree-shape distance
template<typename S, typename NarrowPhaseSolver>
class OcTreeShapeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  OcTreeShapeDistanceTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;
    
    otsolver = NULL;
  }

  FCL_REAL BVTesting(int, int) const
  {
    return -1;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeShapeDistance(model1, *model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const S* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for mesh-octree collision
template<typename BV, typename NarrowPhaseSolver>
class MeshOcTreeCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  MeshOcTreeCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVTesting(int, int) const
  {
    return false;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeMeshIntersect(model2, model1, tf2, tf1, request, *result);
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  Transform3f tf1, tf2;
    
  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for octree-mesh collision
template<typename BV, typename NarrowPhaseSolver>
class OcTreeMeshCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  OcTreeMeshCollisionTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;

    otsolver = NULL;
  }

  bool BVTesting(int, int) const
  {
    return false;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeMeshIntersect(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  Transform3f tf1, tf2;
    
  const OcTreeSolver<NarrowPhaseSolver>* otsolver;
};

/// @brief Traversal node for mesh-octree distance
template<typename BV, typename NarrowPhaseSolver>
class MeshOcTreeDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  MeshOcTreeDistanceTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;
    
    otsolver = NULL;
  }

  FCL_REAL BVTesting(int, int) const
  {
    return -1;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeMeshDistance(model2, model1, tf2, tf1, request, *result);
  }

  const BVHModel<BV>* model1;
  const OcTree* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;

};

/// @brief Traversal node for octree-mesh distance
template<typename BV, typename NarrowPhaseSolver>
class OcTreeMeshDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  OcTreeMeshDistanceTraversalNode()
  {
    model1 = NULL;
    model2 = NULL;
    
    otsolver = NULL;
  }

  FCL_REAL BVTesting(int, int) const
  {
    return -1;
  }

  void leafTesting(int, int) const
  {
    otsolver->OcTreeMeshDistance(model1, model2, tf1, tf2, request, *result);
  }

  const OcTree* model1;
  const BVHModel<BV>* model2;

  const OcTreeSolver<NarrowPhaseSolver>* otsolver;

};



}

#endif
