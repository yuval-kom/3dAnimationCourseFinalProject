#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <iostream>
#include <set>

using namespace Eigen;
using namespace igl;

SandBox::SandBox()
{


}

void SandBox::Init(const std::string& config)
{
    std::string item_name;
    std::ifstream nameFileout;
    doubleVariable = 0;
    nameFileout.open(config);
    if (!nameFileout.is_open())
    {
        std::cout << "Can't open file " << config << std::endl;
    }
    else
    {

        while (nameFileout >> item_name)
        {
            std::cout << "openning " << item_name << std::endl;
            load_mesh_from_file(item_name);

            parents.push_back(-1);
            data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
            data().show_overlay_depth = false;
            data().point_size = 10;
            data().line_width = 2;
            data().set_visible(false, 1);
            initDataStructure(data().V, data().F);

        }
        nameFileout.close();
    }
    MyTranslate(Eigen::Vector3d(0, 0, -1), true);

    data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

SandBox::~SandBox()
{

}


void SandBox::calc_vertex_cost(const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXi& E)
{
    vector<Eigen::Matrix4d> vertexsQTmp;
    vertexsQTmp.resize(V.rows(), Matrix4d::Zero());

    for (int f = 0; f < F.rows(); f++)
    {
        RowVectorXd fNormal = data().F_normals.row(f).normalized();
        for (int i = 0; i < 3; i++) {
            Matrix4d Qtmp = vertexsQTmp.at(F(f, i));
            RowVectorXd v = V.row(F(f, i));
            Matrix4d Kp;
            double d = -1.0 * fNormal.dot(v);
            RowVector4d p;
            p << fNormal, d;
            Kp = p.transpose() * p;
            Qtmp = Qtmp + Kp;
            vertexsQTmp.at(F(f, i)) = Qtmp;
        }
    }
    vertexsQ.push_back(vertexsQTmp);

}

void SandBox::initDataStructure(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    const auto& calc_edge_cost = [&](const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi&,
        const Eigen::MatrixXi& E,
        const Eigen::VectorXi&,
        const Eigen::MatrixXi&,
        const Eigen::MatrixXi&,
        double& cost, Eigen::RowVectorXd& p)->void
    {
        Matrix4d newQ = vertexsQ[selected_data_index].at(E(e, 0)) + vertexsQ[selected_data_index].at(E(e, 1));
        Matrix4d newQDerivative = newQ;
        Vector4d vecToMul;
       
        vecToMul << 0, 0, 0, 1;
        newQDerivative.row(3) = vecToMul;

        VectorXd newV = newQDerivative.inverse() * vecToMul;
        cost = newV.transpose() * newQ * newV;
        newV.conservativeResize(3);
        p = newV;  
    };

    MatrixXi* Etmp = new MatrixXi(), * EFtmp = new MatrixXi(), * EItmp = new MatrixXi();
    VectorXi* EMAPtmp = new VectorXi();
    SandBox::PriorityQueue* Qtmp = new SandBox::PriorityQueue;
    vector<SandBox::PriorityQueue::iterator> QitTmp;

    edge_flaps(F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);

    QitTmp.resize(Etmp->rows());
    C.push_back(new MatrixXd(Etmp->rows(), V.cols()));
    VectorXd costs(Etmp->rows());
    Qtmp->clear();

    calc_vertex_cost(V, F, *Etmp);
    
    for (int e = 0; e < Etmp->rows(); e++) {
        double cost = e;
        RowVectorXd p(1, 3);
        calc_edge_cost(e, V, F, *Etmp, *EMAPtmp, *EFtmp, *EItmp, cost, p);
        C[selected_data_index]->row(e) = p;
        QitTmp[e] = Qtmp->insert(pair<double, int>(cost, e)).first;
    }

    EMAP.push_back(EMAPtmp);
    E.push_back(Etmp);
    EF.push_back(EFtmp);
    EI.push_back(EItmp);
    Q.push_back(Qtmp);
    Qit.push_back(QitTmp);
    num_collapsed.push_back(0);

    currCollapseEdge.push_back(-1);
}


void SandBox::printInfo(const int e, double& cost,
    Eigen::RowVectorXd& p) {
    cout << "edge " << e << ", cost = " << cost << " new v position (" << p[0] << "," << p[1] << "," << p[2] << ")" << endl;

}

void SandBox::simplification()
{
    const auto& calc_edge_cost = [&](const int e,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi&,
        const Eigen::MatrixXi& E,
        const Eigen::VectorXi&,
        const Eigen::MatrixXi&,
        const Eigen::MatrixXi&,
        double& cost, Eigen::RowVectorXd& p)->void
    {
        
        Matrix4d newQ = vertexsQ[selected_data_index].at(E(e, 0)) + vertexsQ[selected_data_index].at(E(e, 1));

        Matrix4d newQDerivative = newQ;
        Vector4d vecToMul;

        newQDerivative.row(3) << 0, 0, 0, 1;
        vecToMul << 0, 0, 0, 1;

        VectorXd newV = newQDerivative.inverse() * vecToMul;
        cost = newV.transpose() * newQ * newV;
        newV.conservativeResize(3);
        p = newV;

        if (currCollapseEdge[selected_data_index] >= 0) {
            vertexsQ[selected_data_index].at(E(e, 0)) = newQ;
            currCollapseEdge[selected_data_index] = -1;
            printInfo(e, cost, p);
        }
    };

    if (!Q[selected_data_index]->empty())
    {

        bool something_collapsed = false;
        int numToCollapse = std::ceil(0.05 * Q[selected_data_index]->size());

        for (int j = 0; j < numToCollapse; j++)
        {
            std::pair<double, int> minEdge = *Q[selected_data_index]->begin();
            if (minEdge.first != std::numeric_limits<double>::infinity())
            {
                currCollapseEdge[selected_data_index] = minEdge.second;
       
            }
            else
            {
                currCollapseEdge[selected_data_index] = -1;
            }
            if (!collapse_edge(calc_edge_cost, data().V, data().F,
                *E[selected_data_index], *EMAP[selected_data_index], *EF[selected_data_index], *EI[selected_data_index], *Q[selected_data_index], Qit[selected_data_index], *C[selected_data_index])) {

                break;
            }
            
            something_collapsed = true;
            num_collapsed.at(selected_data_index)++;
        }

        if (something_collapsed)
        {
            data().set_mesh(data().V, data().F);
            data().set_face_based(true);
            data().dirty = 157;

        }
    }
}

void SandBox::Animate()
{
    if (isActive)
    {



    }
}