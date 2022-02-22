#pragma once

//#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/joint.h"
#include "igl/aabb.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <time.h>

using namespace std;
class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	void initDataStructure(Eigen::MatrixXd&, Eigen::MatrixXi&);
	
	
	std::vector<igl::opengl::Joint*> joints;
	Eigen::VectorXi P; //parents for joints
	vector<Eigen::Vector3d> JointsPoses;
	


	Eigen::Matrix3d CalcParentsRot(int indx);
	Eigen::Vector3d getJoint(int indx);
	Eigen::Matrix4d CalcParentsTransJoint(int indx);

	void set_tip_pos();
	void set_destination_pos(Eigen::Vector3d dir);

	void add_joints();
	void drawJoints();
	double calc_related_distance(int i);
	void add_weights();
	
	void Skinning();
	void moveJoints();
	double sum(int i);
	void normalizeToOne(int i);

private:

	//Project
	Eigen::MatrixXd W;
	typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>RotationList;
	Eigen::RowVector3d sea_green = Eigen::RowVector3d((70.0 / 255), 252.0 / 255., 167 / 255.);
	//maybe delete M and P (??)
	Eigen::MatrixXd C, U, M;
	Eigen::MatrixXi BE;
	
	std::vector<RotationList > poses; // rotations of joints for animation
	double anim_t = 0.0;
	double anim_t_dir = 0.015;
	bool use_dqs = false;
	bool recompute = true;
	RotationList vQ; //rotation of joints
	vector<Eigen::Vector3d> vT; //translation of joints

	void scaleSnake();
	void calcTreeForSnakeHead(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

	void setTexture(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
	
	bool checkConditions(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);
	bool collisionDetec(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);

	bool isTooFar();
	
	void Animate();
	
};
