#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <iostream>
#include <set>

using namespace Eigen;
using namespace igl;
using namespace std;

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
            data().show_overlay_depth = true;
            data().show_texture = true;
            data().point_size = 10;
            data().line_width = 2;
            data().show_overlay = 1;
            data().show_lines = 1;
            data().show_overlay_depth = 2;
            data().set_visible(false, 1);
            parents.push_back(-1);
            
           // //Assignmet1
           // initDataStructure(data().V, data().F);
           // //Assignemt 2
           //data().MyTranslate(Eigen::Vector3d(pos, 0, -1), true);
           // pos = pos + 1.8;
           // drawBox(&trees[selected_data_index]->m_box, selected_data_index);

        }
        nameFileout.close();
    }

    MyTranslate(Eigen::Vector3d(0, 0, -14), true);
    //MyTranslate(Eigen::Vector3d(0, 0, -1), true);
    data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

    //Assignment3
    setInitialPosition();
    

}

SandBox::~SandBox()
{
    

}

void SandBox::setInitialPosition() {

    data_list[0].MyTranslate(Vector3d(5, 0, 0), true);
    for (size_t i = 1; i <= links_number; i++) {
        data_list[i].MyTranslate(Vector3d(0, 0, link_Len), false);
        data_list[i].SetCenterOfRotation(Eigen::Vector3d(0, 0, -0.8));
        data_list[i].add_points(Eigen::RowVector3d(0, 0, -0.8), Eigen::RowVector3d(0, 0, 1));
        drawAxis(i);
        if(i > 1)
            parents[i] = i - 1;
    }
    
    
}

void SandBox::drawAxis(size_t i){
    data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0, 0.8), Eigen::RowVector3d(1.6, 0, 0.8), Eigen::RowVector3d(1, 0, 0)); // x axis - red
    data_list[i].add_edges(Eigen::RowVector3d(0, -1.6, 0.8), Eigen::RowVector3d(0, 1.6, 0.8), Eigen::RowVector3d(0, 1, 0)); // y axis - green
    data_list[i].add_edges(Eigen::RowVector3d(0, 0,-0.8), Eigen::RowVector3d(0, 0, 2.4), Eigen::RowVector3d(0, 0, 1)); // z axis - blue
}

void SandBox::set_destination_pos() {
    
    destination_position = (data_list[0].MakeTransd() * Vector4d(0,0, 0, 1)).head(3);

}

void SandBox::set_tip_pos() {  
    tip_position = (CalcParentsTrans(links_number) * data_list[links_number].MakeTransd() * Vector4d(0, 0, (link_Len / 2),1)).head(3);
}

Vector3d SandBox::getJoint(int indx) {
    Vector3d joint =  data_list[1].GetRotation() * Vector3d(0, 0, -link_Len / 2);
    for (size_t i = 1; i <= indx; i++) {
        Matrix3d R = CalcParentsRot(i);
        joint = joint + R * Vector3d(0, 0, link_Len);
    }

    return joint;
}


Eigen::Matrix3d SandBox::CalcParentsRot(int indx)
{
    Eigen::Matrix3d prevRot = Eigen::Matrix3d::Identity();

    for (int i = indx; parents[i] >= 0; i = parents[i])
    {
        prevRot = data_list[i].GetRotation() * prevRot;
    }

    return prevRot;
}

bool SandBox::isTooFar() {
    Vector3d FirstLink = (data_list[1].MakeTransd() * Vector4d(0, 0, -(link_Len/2),1)).head(3);
    double distance = (destination_position - FirstLink).norm();
    if (distance > links_number * link_Len)
        return true;
    return false;
}

void SandBox::IK_solver(){

    set_destination_pos();
    set_tip_pos();
    if (isTooFar())
    {
        cout << "cannot reach" << endl;
        return;
    }
    for (size_t i = links_number; i > 0; i--) {
        
        Vector3d E = tip_position;
        Vector3d D = destination_position;
        Vector3d R = getJoint(i);
        Vector3d RE = (E - R).normalized();
        Vector3d RD = (D - R).normalized();

        double distance = (D - E).norm();
        if (distance < 0.1) {
            isActive = false;
            setEulerAngles();
            std::cout << "distance " << distance << endl;
            return;
        }
        
        double cos_angle = RE.dot(RD);
        if (cos_angle < -1) {
            cos_angle = -1;
        }
        else if (cos_angle > 1) {
            cos_angle = 1;
        }
        
        Eigen::Vector3d vecToRotate = RE.cross(RD);
        double angle = acos(cos_angle)/10;

        data_list[i].RotateInSystem(vecToRotate, angle);
        set_tip_pos();
    }

}
void SandBox::setEulerAngles() {

    for (size_t i = 1; i <= links_number; i++) {
        double phi = data_list[i].GetRotation().eulerAngles(2, 0, 2)[0];
        double theta = data_list[i].GetRotation().eulerAngles(2, 0, 2)[1];
        double  psi = data_list[i].GetRotation().eulerAngles(2, 0, 2)[2];

        Matrix3d A1;
        Matrix3d A2;
        Matrix3d A3;
        A1 << cos(phi), -sin(phi), 0,
            sin(phi), cos(phi), 0,
            0, 0, 1;

        A2 << 1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);

        A3 << cos(psi), -sin(psi), 0,
            sin(psi), cos(psi), 0,
            0, 0, 1;


        data_list[i].MyRotate(data_list[i].GetRotation().transpose());
        data_list[i].SetRotation(A1*A2 *A3);
        
    }
  
}

void SandBox::printEulerAngles(){

        double phi = data().GetRotation().eulerAngles(2, 0, 2)[0];
        double theta = data().GetRotation().eulerAngles(2, 0, 2)[1];

        Matrix3d A1;
        Matrix3d A2;
        A1 << cos(phi), -sin(phi), 0,
            sin(phi), cos(phi), 0,
            0, 0, 1;

        A2 << 1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);

        cout << " EulerAngles matrix: " << endl;
        cout << "phi: " << A1 << endl;
        cout << "theta: " << A2 << endl;

}


void SandBox::setPositions() {
    set_tip_pos();

    for (int i = 1; i <= links_number; i++) {
       positions[i -1] = getJoint(i);
    }
    startPos = positions[0];
    positions[links_number] = tip_position;
}

//from linknum to 1
void SandBox::forward() {
    tmpPositions[links_number] = destination_position;
    for (signed int i = links_number - 1; i >= 0; i--){
            double ri = (tmpPositions[i + 1] - tmpPositions[i]).norm();
            double lambda = 1.6 / ri;
            Vector3d newPos = (((1-lambda) * tmpPositions[i + 1]) + (lambda * tmpPositions[i])).normalized();
            tmpPositions[i] = newPos;
    }
}

//from 1 to linknum
void SandBox::backward() {
    tmpPositions[0] = startPos;
    for (size_t i = 0; i < links_number; i++) {
        double ri = (tmpPositions[i + 1] - tmpPositions[i]).norm();
        double lambda = 1.6 / ri;
        Vector3d newPos = (((1 - lambda) * tmpPositions[i]) + (lambda * tmpPositions[i+1])).normalized();

        tmpPositions[i+1] = newPos;
    }
}

void SandBox::RotateJoints() {
    for (size_t i = 0; i < links_number; i++) {

        Vector3d oldVector = (positions[i +1] - positions[i]).normalized();
        Vector3d newVector = (tmpPositions[i + 1] - tmpPositions[i]).normalized();

        double cos_angle = newVector.dot(oldVector);
        if (cos_angle < -1) {
            cos_angle = -1;
        }
        else if (cos_angle > 1) {
            cos_angle = 1;
        }
        Eigen::Vector3d vecToRotate = newVector.cross(oldVector);
        double angle = acos(cos_angle) / 10;

        data_list[i+1].MyRotate((CalcParentsRot(i+1) * data_list[i+1].GetRotation()).inverse() * (-vecToRotate), angle);
        setPositions();
    }
}

void SandBox::fabrik() {
    
    double distance = (tip_position - destination_position).norm();
    if (isTooFar())
    {
        cout << "cannot reach" << endl;
        isActive = false;
        return;
    }
    if (distance > 0.1)
    {
        tmpPositions = positions;
        forward();
        backward();
        RotateJoints();
    }
    else {
        cout << "distance: " << distance << endl;
        isActive = false;
        return;
    }
   
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

void SandBox::drawBox(AlignedBox<double, 3>* box , size_t index) {

    VectorXd c = box->center();
    Eigen::MatrixXd V_box(8, 3);
    V_box <<
       c.x() - box->sizes()(0)/2, c.y() - box->sizes()(1)/2, c.z() - box->sizes()(2)/2,
       c.x() - box->sizes()(0)/2, c.y() - box->sizes()(1)/2, c.z() + box->sizes()(2)/2,
       c.x() - box->sizes()(0)/2, c.y() + box->sizes()(1)/2, c.z() - box->sizes()(2)/2,
       c.x() + box->sizes()(0)/2, c.y() - box->sizes()(1)/2, c.z() - box->sizes()(2)/2,
       c.x() - box->sizes()(0)/2, c.y() + box->sizes()(1)/2, c.z() + box->sizes()(2)/2,
       c.x() + box->sizes()(0)/2, c.y() + box->sizes()(1)/2, c.z() - box->sizes()(2)/2,
       c.x() + box->sizes()(0)/2, c.y() - box->sizes()(1)/2, c.z() + box->sizes()(2)/2,
       c.x() + box->sizes()(0)/2, c.y() + box->sizes()(1)/2, c.z() + box->sizes()(2)/2;

    // Edges of the bounding box
    Eigen::MatrixXi E_box(12, 2);
    E_box <<
        0, 1,
        0, 2,
        1, 4,
        3, 0,
        1, 6,
        2, 4,
        2, 5,
        6, 3,
        5, 3,
        7, 5,
        7, 6,
        7, 4;

    data_list[index].add_points(V_box, Eigen::RowVector3d(1, 0, 0));

    for (unsigned i = 0; i < E_box.rows(); ++i)
        data_list[index].add_edges
        (
            V_box.row(E_box(i, 0)),
            V_box.row(E_box(i, 1)),
            Eigen::RowVector3d(1, 0, 0)
        );
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

    //Assingment 2
    AABB<MatrixXd, 3>* treeTmp = new AABB<MatrixXd, 3>;
    treeTmp->init(data().V, data().F);
    trees.push_back(treeTmp);
    subTrees.push_back(treeTmp);
    velocities.push_back(Eigen::Vector3d(-0.009, 0, 0));
    
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


bool SandBox::checkConditions(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i) {

    AlignedBox<double, 3> boxA = Atree->m_box;
    AlignedBox<double, 3> boxB = Btree->m_box;

    size_t currData = selected_data_index;

    Vector4d C0;
    C0 << boxA.center()(0), boxA.center()(1), boxA.center()(2), 1;
    VectorXd center0 = data_list[currData].MakeTransd() * C0 ;

    Vector4d C1;
    C1 << boxB.center()(0), boxB.center()(1), boxB.center()(2), 1;
    VectorXd center1 = data_list[i].MakeTransd() * C1;

    MatrixXd A = data_list[currData].GetRotation();
    MatrixXd B = data_list[i].GetRotation();
    VectorXd A0 = A.row(0);
    VectorXd A1 = A.row(1);
    VectorXd A2 = A.row(2);

    VectorXd B0 = B.row(0);
    VectorXd B1 = B.row(1);
    VectorXd B2 = B.row(2);

    double a0 = boxA.sizes()[0]/2;
    double a1 = boxA.sizes()[1]/2;
    double a2 = boxA.sizes()[2]/2;

    double b0 = boxB.sizes()[0]/2;
    double b1 = boxB.sizes()[1]/2;
    double b2 = boxB.sizes()[2]/2;

    RowVector3d D = { center1(0) - center0(0), center1(1) - center0(1), center1(2) - center0(2) };
   // D.conservativeResize(3);
    MatrixXd C = A.transpose() * B;
    Vector3d aIndex;
    aIndex << a0, a1, a2;
    Vector3d bIndex;
    bIndex << b0, b1, b2;

    VectorXd Aindex[3] = { A0, A1, A2 };
    VectorXd Bindex[3] = { B0, B1, B2 };

    //bool isCollided = true;
    double R0 = 0;
    double R1 = 0;
    double R = 0;

    //condition 1-6:
    for (size_t i = 0; i < 3; i++) {
        double R0 = aIndex[i];
        double R1 = b0 * (abs(C(i, 0))) + b1 * (abs(C(i, 1))) + b2 * (abs(C(i, 2)));
        double R = abs(Aindex[i].dot(D));
        if (R > R0 + R1) {
            return false;
        }
    }

    for (size_t i = 0; i < 3; i++) {
        double R0 = a0 * (abs(C(0, i))) + a1 * (abs(C(1, i))) + a2 * (abs(C(2, i)));
        double R1 = bIndex[i];
        double R = abs(Bindex[i].dot(D));
        if (R > R0 + R1) {
            return false;
        }
    }

    //condition 7-15:

    //A0xB0
    R0 = a1 * abs(C(2, 0)) + a2 * abs(C(1, 0));
    R1 = b1 * abs(C(0, 2)) + b2 * abs(C(0, 1));
    R = abs(C(1, 0) * A2.dot(D) - C(2, 0) * A1.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A0xB1
    R0 = a1 * abs(C(2, 1)) + a2 * abs(C(1, 1));
    R1 = b0 * abs(C(0, 2)) + b2 * abs(C(0, 0));
    R = abs(C(1, 1) * A2.dot(D) - C(2, 1) * A1.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A0xB2
    R0 = a1 * abs(C(2, 2)) + a2 * abs(C(1, 2));
    R1 = b0 * abs(C(0, 1)) + b1 * abs(C(0, 0));
    R = abs(C(1, 2) * A2.dot(D) - C(2, 2) * A1.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A1xB0
    R0 = a0 * abs(C(2, 0)) + a2 * abs(C(0, 0));
    R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 1));
    R = abs(C(2, 0) * A0.dot(D) - C(0, 0) * A2.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A1xB1
    R0 = a0 * abs(C(2, 1)) + a2 * abs(C(0, 1));
    R1 = b1 * abs(C(1, 2)) + b2 * abs(C(1, 0));
    R = abs(C(2, 1) * A0.dot(D) - C(0, 1) * A2.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A1xB2
    R0 = a0 * abs(C(2, 2)) + a2 * abs(C(0, 2));
    R1 = b1 * abs(C(1, 1)) + b1 * abs(C(1, 0));
    R = abs(C(2, 2) * A0.dot(D) - C(0, 2) * A2.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A2xB0
    R0 = a0 * abs(C(1, 0)) + a1 * abs(C(0, 0));
    R1 = b1 * abs(C(2, 2)) + b2 * abs(C(2, 1));
    R = abs(C(0, 0) * A1.dot(D) - C(1, 0) * A0.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    //A2xB1
    R0 = a0 * abs(C(1, 1)) + a1 * abs(C(0, 1));
    R1 = b0 * abs(C(2, 2)) + b2 * abs(C(2, 0));
    R = abs(C(0, 1) * A1.dot(D) - C(1, 1) * A0.dot(D));
    if (R > R0 + R1){
        return false;
    }

    //A2xB2
    R0 = a0 * abs(C(1, 2)) + a1 * abs(C(0, 2));
    R1 = b0 * abs(C(2, 1)) + b1 * abs(C(2, 0));
    R = abs(C(0, 2) * A1.dot(D) - C(1, 2) * A0.dot(D));
    if (R > R0 + R1) {
        return false;
    }

    return true;
}
bool SandBox::collisionDetec(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i)
{    
    bool isCollided = checkConditions(Atree, Btree, i);
    
    if (isCollided) {
        if (Atree->is_leaf() && Btree->is_leaf()) {
            subTrees[selected_data_index] = Atree;
            subTrees[i] = Btree;
            return true;
        }
        else if (Atree->is_leaf()) {
            return((collisionDetec(Atree, Btree->m_left,i)) || collisionDetec(Atree, Btree->m_right,i));
        }
        else if(Btree->is_leaf()){
            return((collisionDetec(Atree->m_left, Btree,i)) || collisionDetec(Atree->m_right, Btree,i));
         }
        else {
            return((collisionDetec(Atree->m_left, Btree->m_left,i)) || collisionDetec(Atree->m_left, Btree->m_right,i)
                || collisionDetec(Atree->m_right, Btree->m_left,i) || collisionDetec(Atree->m_right, Btree->m_right,i));
        }
    }
    return false;
}

void SandBox::translateData(Eigen::Vector3d dir) {
    velocities[selected_data_index] = dir;
}

void SandBox::Animate()
{
    //Assignment2
    //
    
    if (isActive)
    {
        //IK_solver();
        //set_tip_pos();
        //setPositions();
        fabrik();
            //Assignment2
            /*for (size_t i = 0; i < data_list.size(); i++) {
                if (i != selected_data_index){
                    if (collisionDetec(trees[selected_data_index], trees[i], i)) {
                        drawBox(&subTrees[selected_data_index]->m_box, selected_data_index);
                        drawBox(&subTrees[i]->m_box, i);
                    }
                    else
                    {
                      data().MyTranslate(velocities[selected_data_index], true);
                    }
                }
            }*/
    }
}