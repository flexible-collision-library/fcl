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

#include "fcl/traversal_node_bvhs.h"
#include "fcl/collision_node.h"
#include "fcl/simple_setup.h"
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <cstdlib>
#include <cstdio>
#include <boost/timer.hpp>

#define _strdup strdup

using namespace fcl;

unsigned int n_dcd_samples = 10;

bool loadPLYFile(const std::string& fileName, float scalarFactor, std::vector<Vec3f>& vert_list,
                 std::vector<Triangle>& tri_list);

void lionTest();

void ballTest();

void lionTest_DCD();

void ballTest_DCD();

int main()
{
  lionTest();
  ballTest();
  lionTest_DCD();
  ballTest_DCD();

  return 1;
}

void lionTest()
{
  std::vector<Vec3f> p;
  std::vector<Triangle> t;


  std::string base = std::string("test/deformable_data/breakinglion/");

  BVHModel<AABB> m;
  m.bv_splitter.reset(new BVSplitter<AABB>(SPLIT_METHOD_MEDIAN));

  double timing = 0;

  for(int i = 16; i <= 60; ++i)
  {
    char buffer[10];
    sprintf(buffer, "%d", i);
    std::string filename = base + std::string(buffer) + std::string(".ply");
    loadPLYFile(filename, 1.0, p, t);
    // std::cout << p.size() << " " << t.size() << std::endl;

    boost::timer timer;

    if(i == 16)
    {
      m.beginModel();
      m.addSubModel(p, t);
      m.endModel();
    }
    else
    {
      m.beginUpdateModel();
      m.updateSubModel(p);
      m.endUpdateModel(true, true);
    }

    SimpleTransform tf1, tf2;

    MeshCollisionTraversalNode<AABB> node;
    initialize(node, m, tf1, m, tf2);
    selfCollide(&node);

    timing += timer.elapsed();

    // if(node.pairs.size() > 0) std::cout << "collision" << std::endl;
    // else std::cout << "collision free" << std::endl;
  }

  std::cout << "timing: " << timing << " sec" << std::endl;
}


void lionTest_DCD()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;

  std::string base = std::string("test/deformable_data/breakinglion/");


  double timing = 0;

  for(int i = 17; i <= 60; ++i)
  {
    {
      char buffer[10];
      sprintf(buffer, "%d", i - 1);
      std::string filename = base + std::string(buffer) + std::string(".ply");
      loadPLYFile(filename, 1.0, p1, t1);
    }

    {
      char buffer[10];
      sprintf(buffer, "%d", i);
      std::string filename = base + std::string(buffer) + std::string(".ply");
      loadPLYFile(filename, 1.0, p2, t2);
    }

    std::vector<Vec3f> p(p1.size());

    boost::timer timer;

    for(unsigned int n = 0; n <= n_dcd_samples; ++n)
    {
      // std::cout << i << " " << n << std::endl;
      double delta = n / (double)n_dcd_samples;
      for(unsigned int j = 0; j < p.size(); ++j)
      {
        p[j] = p1[j] * (1 - delta) + p2[j] * delta;
      }

      BVHModel<AABB> m;
      m.bv_splitter.reset(new BVSplitter<AABB>(SPLIT_METHOD_MEDIAN));

      m.beginModel();
      m.addSubModel(p, t1);
      m.endModel();

      SimpleTransform tf1, tf2;

      MeshCollisionTraversalNode<AABB> node;
      initialize(node, m, tf1, m, tf2);
      selfCollide(&node);

      if(node.pairs.size() > 0) break;
    }

    timing += timer.elapsed();

    //if(node.pairs.size() > 0) std::cout << "collision" << std::endl;
    //else std::cout << "collision free" << std::endl;
  }

  std::cout << "timing: " << timing << " sec" << std::endl;
}


void ballTest()
{
  std::vector<Vec3f> p;
  std::vector<Triangle> t;


  std::string base = std::string("test/deformable_data/balls16/balls16_");

  BVHModel<AABB> m;
  m.bv_splitter.reset(new BVSplitter<AABB>(SPLIT_METHOD_MEDIAN));

  double timing = 0;

  for(int i = 0; i <= 75; ++i)
  {
    char buffer[10];
    sprintf(buffer, "%d", i);
    std::string filename = base + std::string(buffer) + std::string(".ply");
    loadPLYFile(filename, 1.0, p, t);
    // std::cout << p.size() << " " << t.size() << std::endl;

    boost::timer timer;

    if(i == 0)
    {
      m.beginModel();
      m.addSubModel(p, t);
      m.endModel();
    }
    else
    {
      m.beginUpdateModel();
      m.updateSubModel(p);
      m.endUpdateModel(true, true);
    }

    SimpleTransform tf1, tf2;

    MeshCollisionTraversalNode<AABB> node;
    initialize(node, m, tf1, m, tf2);
    selfCollide(&node);

    timing += timer.elapsed();

    // if(node.pairs.size() > 0) std::cout << "collision" << std::endl;
    // else std::cout << "collision free" << std::endl;
  }

  std::cout << "timing: " << timing << " sec" << std::endl;
}


void ballTest_DCD()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;


  std::string base = std::string("test/deformable_data/balls16/balls16_");

  double timing = 0;

  for(int i = 1; i <= 75; ++i)
  {
    {
      char buffer[10];
      sprintf(buffer, "%d", i - 1);
      std::string filename = base + std::string(buffer) + std::string(".ply");
      loadPLYFile(filename, 1.0, p1, t1);
    }

    {
      char buffer[10];
      sprintf(buffer, "%d", i);
      std::string filename = base + std::string(buffer) + std::string(".ply");
      loadPLYFile(filename, 1.0, p2, t2);
    }

    std::vector<Vec3f> p(p1.size());

    boost::timer timer;

    for(unsigned int n = 0; n <= n_dcd_samples; ++n)
    {
      // std::cout << i << " " << n << std::endl;
      double delta = n / (double)n_dcd_samples;
      for(unsigned int j = 0; j < p.size(); ++j)
      {
        p[j] = p1[j] * (1 - delta) + p2[j] * delta;
      }

      BVHModel<AABB> m;
      m.bv_splitter.reset(new BVSplitter<AABB>(SPLIT_METHOD_MEDIAN));

      m.beginModel();
      m.addSubModel(p, t1);
      m.endModel();

      SimpleTransform tf1, tf2;

      MeshCollisionTraversalNode<AABB> node;
      initialize(node, m, tf1, m, tf2);
      selfCollide(&node);

      if(node.pairs.size() > 0) break;
    }

    timing += timer.elapsed();

    //if(node.pairs.size() > 0) std::cout << "collision" << std::endl;
    //else std::cout << "collision free" << std::endl;
  }

  std::cout << "timing: " << timing << " sec" << std::endl;
}

bool loadPLYFile(const std::string& fileName, float scalarFactor, std::vector<Vec3f>& vert_list,
                 std::vector<Triangle>& tri_list)
{
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(fileName,
        0);

  if(!scene) return false;

  int mNumMeshes = scene->mNumMeshes;

  int numVertices = 0;
  int numTriangles = 0;
  for(int i = 0; i < mNumMeshes; ++i)
  {
    aiMesh* mesh = scene->mMeshes[i];
    numVertices += mesh->mNumVertices;
    numTriangles += mesh->mNumFaces;
  }

  vert_list.resize(numVertices);
  tri_list.resize(numTriangles);



  int v_index = 0;
  int t_index = 0;

  for(int i = 0; i < mNumMeshes; ++i)
  {
    aiMesh* mesh = scene->mMeshes[i];

    for(int j = 0 ; j < mesh->mNumVertices; ++j)
    {
      vert_list[v_index + j].setValue(mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z);
    }


    for(int j = 0; j < mesh->mNumFaces; ++j)
    {
      tri_list[t_index + j].set(mesh->mFaces[j].mIndices[0] + v_index,
                                mesh->mFaces[j].mIndices[1] + v_index,
                                mesh->mFaces[j].mIndices[2] + v_index);
    }
    v_index += mesh->mNumVertices;
    t_index += mesh->mNumFaces;
  }

  return true;
}
