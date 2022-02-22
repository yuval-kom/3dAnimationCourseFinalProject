// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/jet.h>


#include <Windows.h>
#include <mmsystem.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0; 

namespace igl
{
namespace opengl
{
namespace glfw
{

  using namespace Eigen;
  void Viewer::Init(const std::string config)
  {
	  

  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;
    level = 0;
    isDuringLevel = false;
    isWonLevel = false;
  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      
      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;
    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());

  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

 

  void Viewer::AddNewShape(int savedIndx) {

      data().show_overlay_depth = true;
      data().point_size = 10;
      data().line_width = 2;
      data().show_lines = false;
      data().show_overlay_depth = 2;
      data().set_visible(true, 1);
      parents.push_back(-1);
      selected_data_index = savedIndx;
      
      
  }
  void Viewer::moveObjects() {
      for (int i = 1; i < data_list.size(); i++) {
          ViewerData* currData = &data_list[i];
          currData->MyTranslate(currData->direction, false);
          Eigen::RowVector4d objectPosition = (currData->MakeTransd() * Eigen::Vector4d(0, 0, 0, 1));
          if (objectPosition(2) <= 5 || objectPosition(2) > 40) { //z
              currData->direction << currData->direction(0), currData->direction(1), -currData->direction(2);
          }
          if (objectPosition(0) > 10|| objectPosition(0) < -10) { //x
              currData->direction << -currData->direction(0), currData->direction(1), currData->direction(2);
          }

      }
  }
  void Viewer::start_level() {
      score = 0;

      int numToAdd = 0;
      Vector3d dir;
      switch (level)
      {
      case 0:
          numToAdd = 3;
          dir = Vector3d(-0.02,0, 0.02);
          break;
      case 1:
          numToAdd = 4; 
          data_list[0].direction *= 2;
          dir = Vector3d(0.04, 0, -0.04);
          for (int i = 1; i < numToAdd - 1; i++) {
              load_mesh_from_file("C:/AnimationCourseEngine/tutorial/data/cube.obj");
              int currIndex = data_list.size() - 1;
              AddNewShape(currIndex);
              data_list[currIndex].isPrize = false;
              data_list[currIndex].MyTranslate(Eigen::Vector3d(rand() % 5, 0, rand() % 30 + double(i) * 2), false);
              data_list[currIndex].gamePoints = -1;
              data_list[currIndex].tree.init(data_list[currIndex].V, data_list[currIndex].F);
              data_list[currIndex].set_colors(Eigen::RowVector3d(0.6, 0.2, 0.1));
              data_list[currIndex].direction = dir*i;
              dir = -dir;

          }
      }

      ScoreGoal = (numToAdd * 5);
      for (int i = 1; i <= numToAdd; i++) {
          load_mesh_from_file("C:/AnimationCourseEngine/tutorial/data/sphere.obj");
          int currIndex = data_list.size() - 1;
          AddNewShape(currIndex);
          data_list[currIndex].isPrize = true;
          data_list[currIndex].MyTranslate(Eigen::Vector3d(rand()%5,0,rand() % 30 + double(i) * 2), false);
          data_list[currIndex].gamePoints = 5;
          data_list[currIndex].tree.init(data_list[currIndex].V, data_list[currIndex].F);
          data_list[currIndex].direction = dir*i;
          dir = -dir;

          Eigen::MatrixXd C;
          Eigen::VectorXd Z = data_list[currIndex].V.col(2);
          igl::jet(Z, true, C);
          data_list[currIndex].set_colors(C);
      }

      //set to initial positoin
      data_list[0].MyTranslate((data_list[0].MakeTransd().inverse() * Vector4d(0, 0, 0, 1)).head(3), false);
      data_list[0].MyRotate(data_list[0].GetRotation().inverse());
      time(&level_start_time);
      PlaySound(TEXT("C:/AnimationCourseEngine/sounds/GameSound.wav"), NULL, SND_FILENAME | SND_LOOP | SND_ASYNC);
      isDuringLevel = true;
      isActive = true;

  }

  void Viewer::playBiteSound() {
      PlaySound(TEXT("C:/AnimationCourseEngine/sounds/bite.wav"), NULL, SND_FILENAME);
      PlaySound(TEXT("C:/AnimationCourseEngine/sounds/GameSound.wav"), NULL, SND_FILENAME | SND_LOOP | SND_ASYNC);
  }

  void Viewer::resetScene(bool isWon) {
      int numOfObjects = data_list.size();
      for (int i = numOfObjects - 1; i > 0; i--) {
          std::cout << "inn" << std::endl;
          erase_mesh(i);
      }
      if (isWon) {
         std::cout << " Finish level";
         PlaySound(TEXT("C:/AnimationCourseEngine/sounds/WinLevel.wav"), NULL, SND_FILENAME);
         isDuringLevel = false;
         isWonLevel = true;
         level++;
      }
      else {
          PlaySound(TEXT("C:/AnimationCourseEngine/sounds/GameOver.wav"), NULL, SND_FILENAME);
          std::cout << " YOU LOSE!";
          isDuringLevel = false;
          isWonLevel = false;
      }
  }

  

} // end namespace
} // end namespace
}
