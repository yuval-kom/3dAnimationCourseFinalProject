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
class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	void initDataStructure(Eigen::MatrixXd&, Eigen::MatrixXi&);
	void simplification();
	//Assignment2
	void translateData(Eigen::Vector3d dir);

	//Assignmet3
	
	Eigen::Vector3d destination_position = Eigen::Vector3d(5, 0, 0);
	Eigen::Vector3d startPos;

    vector<Eigen::Vector3d> positions;
	vector<Eigen::Vector3d> tmpPositions;
	Eigen::Matrix3d CalcParentsRot(int indx);
	Eigen::Vector3d getJoint(int indx);
	void set_destination_pos();
	void set_tip_pos();
	void IK_solver();
	void printEulerAngles();
	void setEulerAngles();
	void setPositions();
	void backward();
	void forward();
	void RotateJoints();
	void fabrik();


private:
	// Prepare array-based edge data structures and priority queue
	vector<Eigen::VectorXi*> EMAP;
	vector<Eigen::MatrixXi*> E, EF, EI;
	typedef set<std::pair<double, int>> PriorityQueue;
	vector<PriorityQueue*> Q;
	vector<vector<PriorityQueue::iterator>> Qit;
	vector<Eigen::MatrixXd*> C;
	vector<int> num_collapsed;

	vector<vector<Eigen::Matrix4d>> vertexsQ;
	vector<int> currCollapseEdge;

	void calc_vertex_cost(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXi& E);
	void printInfo(const int e, double& cost,Eigen::RowVectorXd& p);

	//Assignmet 2
	double pos = -1;
	vector<igl::AABB<Eigen::MatrixXd, 3>*> trees;
	vector<igl::AABB<Eigen::MatrixXd, 3>*> subTrees;
	
	vector<Eigen::Vector3d> velocities;
	void drawBox(Eigen::AlignedBox<double, 3>* box, size_t index);
	bool checkConditions(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);
	bool collisionDetec(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i);

	//Assignment 3
	void setInitialPosition();
	void drawAxis(size_t i);
	bool isTooFar();
	
	void Animate();
	
};
