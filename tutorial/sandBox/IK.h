#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>

using namespace std;
class IK : public igl::opengl::glfw::Viewer
{
public:
	IK();
	~IK();
	
	Eigen::Vector3d destination_position = Eigen::Vector3d(5, 0, 0);
	void set_destination_pos();
	void set_tip_pos();
	void ccd();
	void printEulerAngles();


private:
	void setInitialPosition();
	void drawAxis(size_t i);
	bool isTooFar();
	
	void Animate();
	
};
