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
	//void simplification();
	//Assignment2
	void translateData(Eigen::Vector3d dir);
	
	
	
	/*Eigen::Vector3d startPos;
    vector<Eigen::Vector3d> positions;
	vector<Eigen::Vector3d> tmpPositions;*/

	
	//void IK_solver();
	////void printEulerAngles();
	////void setEulerAngles();
	//void setPositions();
	//void backward();
	//void forward();
	//void RotateJoints();
	//void fabrik();

	//Project
	
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
	// Prepare array-based edge data structures and priority queue
	/*vector<Eigen::VectorXi*> EMAP;
	vector<Eigen::MatrixXi*> E, EF, EI;
	typedef set<std::pair<double, int>> PriorityQueue;
	vector<PriorityQueue*> Q;
	vector<vector<PriorityQueue::iterator>> Qit;
	vector<Eigen::MatrixXd*> C;
	vector<int> num_collapsed;

	vector<vector<Eigen::Matrix4d>> vertexsQ;
	vector<int> currCollapseEdge;*/

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

	//void calc_vertex_cost(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXi& E);
	//void printInfo(const int e, double& cost,Eigen::RowVectorXd& p);

	//Assignmet 2
	/*double pos = -1;
	vector<igl::AABB<Eigen::MatrixXd, 3>*> trees;
	vector<igl::AABB<Eigen::MatrixXd, 3>*> subTrees;*/
	
	vector<Eigen::Vector3d> velocitiesByLevel;
	
	bool checkConditions(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);
	bool collisionDetec(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);

	//Assignment 3
	//void setInitialPosition();
	void drawAxis(size_t i);
	bool isTooFar();
	
	void Animate();
	
};
