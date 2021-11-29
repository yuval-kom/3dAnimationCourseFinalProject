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
	void Animate();
	
};
