#define BOOST_TEST_MODULE "FCL_XMLDATA"
#include <tinyxml.h>

#include <boost/test/unit_test.hpp>

#include "fcl/intersect.h"
#include "fcl/collision.h"
#include "fcl/BVH/BVH_model.h"
#include <boost/filesystem.hpp>
#include <sstream>


#include "libsvm_classifier.h"
#include "fcl/penetration_depth.h"
#include "fcl_resources/config.h"
#include "test_fcl_utility.h"


using namespace fcl;

static void loadSceneFile(const std::string& filename,
                          std::vector<std::vector<Vec3f> >& points_array,
                          std::vector<std::vector<Triangle> >& triangles_array,
                          std::vector<std::pair<Transform3f, Transform3f> >& motions)
{
  TiXmlDocument doc(filename.c_str());
  if(doc.LoadFile())
  {
    TiXmlHandle hdoc(&doc);
    TiXmlElement* model, *motion;
    model = doc.FirstChildElement("MODEL");
    if(model)
    {
      TiXmlElement* object = model->FirstChildElement("OBJECT");
      int i = 0;
      while(object)
      {
        int object_id = -1;
        object->Attribute("id", &object_id);
        // std::cout << object_id << std::endl;

        std::vector<Vec3f> points;
        std::vector<Triangle> triangles;

        TiXmlElement* grid = object->FirstChildElement("GRID");
        int n_vertices = 0;
        while(grid)
        {
          int grid_id;
          double grid_x, grid_y, grid_z;

          grid->Attribute("id", &grid_id);
          grid->Attribute("x", &grid_x);
          grid->Attribute("y", &grid_y);
          grid->Attribute("z", &grid_z);

          Vec3f p(grid_x, grid_y, grid_z);

          if(grid_id - 1 == (int)points.size())
            points.push_back(p);
          else if(grid_id - 1 < (int)points.size())
            points[grid_id - 1] = p;
          else // if(grid_id - 1 > points.size())
          {
            points.resize(grid_id);
            points.back() = p;
          }
                          
          n_vertices++;
          grid = grid->NextSiblingElement("GRID");
        }

        // std::cout << "#vertices " << n_vertices << std::endl;

        TiXmlElement* tri = object->FirstChildElement("TRIA");
        int n_tris = 0;
        while(tri)
        {
          int tri_id;
          int v1, v2, v3;

          tri->Attribute("id", &tri_id);
          tri->Attribute("g1", &v1);
          tri->Attribute("g2", &v2);
          tri->Attribute("g3", &v3);

          Triangle t(v1-1, v2-1, v3-1);

          if(tri_id - 1 == (int)triangles.size())
            triangles.push_back(t);
          else if(tri_id - 1 < (int)triangles.size())
            triangles[tri_id - 1] = t;
          else
          {
            triangles.resize(tri_id);
            triangles.back() = t;
          }
          
          n_tris++;
          tri = tri->NextSiblingElement("TRIA");
        }

        // std::cout << "#triangles " << n_tris << std::endl;

        if(object_id - 1 == (int)points_array.size())
        {
          points_array.push_back(points);
          triangles_array.push_back(triangles);
        }
        else if(object_id - 1 < (int)points_array.size())
        {
          points_array[object_id - 1] = points;
          triangles_array[object_id - 1] = triangles;
        }
        else
        {
          points_array.resize(object_id);
          triangles_array.resize(object_id);
          points_array.back() = points;
          triangles_array.back() = triangles;
        }

        object = object->NextSiblingElement("OBJECT");
        i++;
      }

      // std::cout << "#objects " << i << std::endl;
    }

    motion = doc.FirstChildElement("MOTION");
    if(motion)
    {
      TiXmlElement* frame = motion->FirstChildElement("FRAME");
      int n_frame = 0;
      while(frame)
      {
        int frame_id;
        double frame_time;
        frame->Attribute("id", &frame_id);
        frame->Attribute("time", &frame_time);


        Vec3f T1, T2;
        Matrix3f R1, R2;
        const char* obj1_pos_string = frame->Attribute("obj1_pos");
        const char* obj2_pos_string = frame->Attribute("obj2_pos");
        const char* obj1_dc_string = frame->Attribute("obj1_dc");
        const char* obj2_dc_string = frame->Attribute("obj2_dc");

        std::stringstream s1_pos(obj1_pos_string);
        s1_pos >> T1[0] >> T1[1] >> T1[2];
        std::stringstream s2_pos(obj2_pos_string);
        s2_pos >> T2[0] >> T2[1] >> T2[2];
        std::stringstream s1_mat(obj1_dc_string);
        for(int j = 0; j < 3; ++j)
          for(int k = 0; k < 3; ++k)
            s1_mat >> R1(j, k);
        std::stringstream s2_mat(obj2_dc_string);
        for(int j = 0; j < 3; ++j)
          for(int k = 0; k < 3; ++k)
            s2_mat >> R2(j, k);


        std::pair<Transform3f, Transform3f> motion = std::make_pair(Transform3f(R1, T1), Transform3f(R2, T2));
        if(frame_id - 1 == (int)motions.size())
          motions.push_back(motion);
        else if(frame_id - 1 < (int)motions.size())
          motions[frame_id - 1] = motion;
        else
        {
          motions.resize(frame_id);
          motions.back() = motion;
        }
        
        
        frame = frame->NextSiblingElement("FRAME");
        n_frame++;
      }

      // std::cout << "#frames " << n_frame << std::endl;
    }
  }
  else
    std::cerr << "Failed to load file " << filename << std::endl;    
}

BOOST_AUTO_TEST_CASE(scene_test)
{
  std::vector<std::vector<Vec3f> > points_array;
  std::vector<std::vector<Triangle> > triangles_array;
  std::vector<std::pair<Transform3f, Transform3f> > motions;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  std::string filename = (path / "scenario-1-2-3/Model_1_Scenario_3.txt").string();
  loadSceneFile(filename, points_array, triangles_array, motions);

  BVHModel<OBBRSS> m1;
  BVHModel<OBBRSS> m2;
  m1.beginModel();
  m1.addSubModel(points_array[0], triangles_array[0]);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(points_array[1], triangles_array[1]);
  m2.endModel();

  
  CollisionResult result0;
  CollisionRequest request0;
  collide(&m1, motions[0].first, &m2, motions[0].second, request0, result0);
  // std::cout << result0.numContacts() << std::endl;

  CollisionResult result1;
  CollisionRequest request1;
  collide(&m1, motions[1].first, &m2, motions[1].second, request1, result1);
  // std::cout << result1.numContacts() << std::endl;
}

static std::string num2string(int i)
{
  std::string result;
  std::ostringstream convert;
  convert << i;
  result = convert.str();
  return result;
}

static void xml2obj(const std::string& in_filename, const std::string& out_filename_base)
{
  std::vector<std::vector<Vec3f> > points_array;
  std::vector<std::vector<Triangle> > triangles_array;
  std::vector<std::pair<Transform3f, Transform3f> > motions;
  loadSceneFile(in_filename, points_array, triangles_array, motions);

  std::size_t n_obj = points_array.size();
  // save objs in local frame
  for(std::size_t i = 0; i < n_obj; ++i)
  {
    std::string out_filenameL = out_filename_base + num2string(i+1) + "L.obj";
    saveOBJFile(out_filenameL.c_str(), points_array[i], triangles_array[i]);
  }

  // save objs in frame 1
  for(std::size_t i = 0; i < n_obj; ++i)
  {
    std::string out_filenameF1 = out_filename_base + num2string(i+1) + "Frame1.obj";
    std::vector<Vec3f> points(points_array[i].size());
    for(std::size_t j = 0; j < points.size(); ++j)
      points[j] = motions[i].first.transform(points_array[i][j]);

    saveOBJFile(out_filenameF1.c_str(), points, triangles_array[i]);
  }

  // save objs in frame 2
  for(std::size_t i = 0; i < n_obj; ++i)
  {
    std::string out_filenameF2 = out_filename_base + num2string(i+1) + "Frame2.obj";
    std::vector<Vec3f> points(points_array[i].size());
    for(std::size_t j = 0; j < points.size(); ++j)
      points[j] = motions[i].second.transform(points_array[i][j]);

    saveOBJFile(out_filenameF2.c_str(), points, triangles_array[i]);
  }
}

static void scenePenetrationTest(const std::string& filename)
{
  std::vector<std::vector<Vec3f> > points_array;
  std::vector<std::vector<Triangle> > triangles_array;
  std::vector<std::pair<Transform3f, Transform3f> > motions;

  loadSceneFile(filename, points_array, triangles_array, motions);

  BVHModel<OBBRSS>* m1 = new BVHModel<OBBRSS>();
  BVHModel<OBBRSS>* m2 = new BVHModel<OBBRSS>();
  m1->beginModel();
  m1->addSubModel(points_array[0], triangles_array[0]);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(points_array[1], triangles_array[1]);
  m2->endModel();

  Transform3f id;
  CollisionObject o1(boost::shared_ptr<CollisionGeometry>(m1), id);
  CollisionObject o2(boost::shared_ptr<CollisionGeometry>(m2), id);

  default_transform_distancer = DefaultTransformDistancer(o1.getCollisionGeometry());
  std::cout << "rotation weights ";
  default_transform_distancer.printRotWeight();
  std::size_t KNN_K = 10;
  LibSVMClassifier<6> classifier;
  
  std::vector<Transform3f> contact_vectors = penetrationDepthModelLearning(&o1, &o2, PDT_GENERAL_EULER, &classifier, 10000, 0, KNN_GNAT, KNN_K);

  classifier.save(filename + "model.txt");

  PenetrationDepthRequest request0(&classifier, default_transform_distance_func);
  request0.contact_vectors = contact_vectors;
  PenetrationDepthResult result0;

  penetrationDepth(m1, motions[0].first, m2, motions[0].second, request0, result0);
  std::cout << "pd value 1 " << result0.pd_value << std::endl;

  PenetrationDepthRequest request1(&classifier, default_transform_distance_func);
  request1.contact_vectors = contact_vectors;
  PenetrationDepthResult result1;
  penetrationDepth(m1, motions[1].first, m2, motions[1].second, request1, result1);
  std::cout << "pd value 2 " << result1.pd_value << std::endl;
}


BOOST_AUTO_TEST_CASE(scene_test_penetration)
{
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  std::string filename1 = (path / "scenario-1-2-3/Model_1_Scenario_1.txt").string();
  scenePenetrationTest(filename1);

  std::string filename2 = (path / "scenario-1-2-3/Model_1_Scenario_2.txt").string();
  scenePenetrationTest(filename2);

  std::string filename3 = (path / "scenario-1-2-3/Model_1_Scenario_3.txt").string();
  scenePenetrationTest(filename3);

  std::string filename4 = (path / "scenario-1-2-3/Model_2_Scenario_1.txt").string();
  scenePenetrationTest(filename4);

  std::string filename5 = (path / "scenario-1-2-3/Model_2_Scenario_2.txt").string();
  scenePenetrationTest(filename5);

  std::string filename6 = (path / "scenario-1-2-3/Model_2_Scenario_3.txt").string();
  scenePenetrationTest(filename6);

  std::string filename7 = (path / "scenario-1-2-3/Model_3_Scenario_1.txt").string();
  scenePenetrationTest(filename7);

  std::string filename8 = (path / "scenario-1-2-3/Model_3_Scenario_2.txt").string();
  scenePenetrationTest(filename8);

  std::string filename9 = (path / "scenario-1-2-3/Model_3_Scenario_3.txt").string();
  scenePenetrationTest(filename9);
  
}


BOOST_AUTO_TEST_CASE(xml2obj_test)
{
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  std::string filename1 = (path / "scenario-1-2-3/Model_1_Scenario_1.txt").string();
  xml2obj(filename1, "Model_1_Scenario_1");

  std::string filename2 = (path / "scenario-1-2-3/Model_1_Scenario_2.txt").string();
  xml2obj(filename2, "Model_1_Scenario_2");
  
  std::string filename3 = (path / "scenario-1-2-3/Model_1_Scenario_3.txt").string();
  xml2obj(filename3, "Model_1_Scenario_3");

  std::string filename4 = (path / "scenario-1-2-3/Model_2_Scenario_1.txt").string();
  xml2obj(filename4, "Model_2_Scenario_1");

  std::string filename5 = (path / "scenario-1-2-3/Model_2_Scenario_2.txt").string();
  xml2obj(filename5, "Model_2_Scenario_2");

  std::string filename6 = (path / "scenario-1-2-3/Model_2_Scenario_3.txt").string();
  xml2obj(filename6, "Model_2_Scenario_3");

  std::string filename7 = (path / "scenario-1-2-3/Model_3_Scenario_1.txt").string();
  xml2obj(filename7, "Model_3_Scenario_1");

  std::string filename8 = (path / "scenario-1-2-3/Model_3_Scenario_2.txt").string();
  xml2obj(filename8, "Model_3_Scenario_2");

  std::string filename9 = (path / "scenario-1-2-3/Model_3_Scenario_3.txt").string();
  xml2obj(filename9, "Model_3_Scenario_3");

}
