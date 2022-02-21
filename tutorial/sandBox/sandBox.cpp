#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include <igl/directed_edge_orientations.h>
#include <igl/forward_kinematics.h>
#include <igl/dqs.h>
#include "Eigen/dense"
#include <functional>
#include <iostream>
#include <set>
#include "igl/png/readPNG.h"
#include <igl/jet.h>
#include <igl/slice.h>
#include <igl/slice_into.h>

using namespace Eigen;
using namespace igl;
using namespace std;
using namespace opengl;

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

    
    data_list[0].MyScale(Vector3d(1, 1, 5));
    add_joints();
    drawJoints();
    initDataStructure(data_list[0].V, data_list[0].F);
    MyTranslate(Eigen::Vector3d(0, 0, -14), true);
    //setVelocity(Eigen::Vector3d(-0.009, 0, 0));
    drawAxis(0);
    //MyTranslate(Eigen::Vector3d(0, 0, -1), true);
    //data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

    //set_destination_pos(Vecto);
}

SandBox::~SandBox()
{
    

}

void SandBox::add_joints() {
    JointsPoses.resize(17);
    joints.resize(17);
    P.resize(17);
    C.resize(17, 3);
    Vector3d currPos = Vector3d(0, 0, -0.8);
    for (int i = 0; i < num_of_joints; i++) {
        Joint* curr_joint = new Joint(currPos, i);
        C.row(i) = currPos;
        //cout << "i: " << i << " pos: " << currPos << endl;
        JointsPoses[i] = currPos;
        joints[i] = curr_joint;
        P[i] = i - 1;
        currPos = currPos + Vector3d(0, 0, 0.1);
    }
}
void SandBox::drawJoints() {

    Eigen::MatrixXd V_box(17, 3);
    for (int i = 0; i < 17; i++) {
        V_box.row(i) << C.row(i).x(), C.row(i).y(), C.row(i).z();
    }

    // Edges between joints
    BE.resize(16, 2);
    //Eigen::MatrixXi E_box(15, 2);
    BE <<
        0, 1,
        1, 2,
        2, 3,
        3, 4,
        4, 5,
        5, 6,
        6, 7,
        7, 8,
        8, 9,
        9, 10,
        10, 11,
        11, 12,
        12, 13,
        13, 14,
        14, 15,
        15, 16;

    data_list[0].set_points(V_box, Eigen::RowVector3d(0, 0, 1));

    data_list[0].set_edges(V_box, BE, Eigen::RowVector3d(1, 0, 1));
}

void SandBox::drawAxis(size_t i) {
    data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0, 0.8), Eigen::RowVector3d(1.6, 0, 0.8), Eigen::RowVector3d(1, 0, 0)); // x axis - red
    data_list[i].add_edges(Eigen::RowVector3d(0, -1.6, 0.8), Eigen::RowVector3d(0, 1.6, 0.8), Eigen::RowVector3d(0, 1, 0)); // y axis - green
    data_list[i].add_edges(Eigen::RowVector3d(0, 0, -0.8), Eigen::RowVector3d(0, 0, 1.6), Eigen::RowVector3d(0, 0, 1)); // z axis - blue
}

double SandBox::calc_related_distance(int i) {
    double sum = 0;
    double distance;
    for (int j = 0; j < joints.size(); j++) {
        distance = abs(joints[j]->position.z() - data_list[0].V.row(i).z());
        if (distance <= 0.1) {
            sum += pow((1 / distance), 4);
        }
    }
    return sum;
}

void SandBox::normalizeToOne(int i) {
    int sumOfRow = sum(i);
    if (sumOfRow == 0) return;
    for (int j = 0; j < W.row(i).size(); j++) {
        W(i, j) = W(i, j)/sumOfRow;
    }
}
double SandBox::sum(int i) {
    double sum = 0;
    for (int j = 0; j < W.row(i).size(); j++) {
        sum += W(i,j);
    }
    return sum;
}
void SandBox::add_weights() {
    double distance;
    for (int i = 0; i < data_list[0].V.rows(); i++) {
        double related_distance = calc_related_distance(i);
        for (int j = 0; j < joints.size() -1 ; j++) {
            distance = abs(joints[j + 1]->position.z() - data_list[0].V.row(i).z());
            //Vector3d vPos;
            //vPos << data_list[0].V.row(i).x(), data_list[0].V.row(i).y(), data_list[0].V.row(i).z();
            //distance = (joints[j]->position - 
            /*if (distance <= 0.1) {
                W(i, j) = 1;
            }
            else
            {
                W(i, j) = 0;

            }*/
            double temp = pow((1 / distance), 4);
            W(i, j) = temp / related_distance;
        }
        normalizeToOne(i);
    }

}

Vector3d SandBox::getJoint(int indx) {
    Vector3d joint = joints[0]->GetRotation() * Vector3d(0, 0, -0.8);
    for (int i = 1; i <= indx; i++) {
        Matrix3d R = CalcParentsRot(i);
        joint = joint + R * Vector3d(0, 0, 0.1);
    }

    return joint;
}

Eigen::Matrix4d SandBox::CalcParentsTransJoint(int indx)
{
    Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

    for (int i = indx; i > 0; i = P[i])
    {
        prevTrans = prevTrans * joints[P[i]]->MakeTransd();
    }

    return prevTrans;
}

Eigen::Matrix3d SandBox::CalcParentsRot(int indx)
{
    Eigen::Matrix3d prevRot = Eigen::Matrix3d::Identity();

    for (int i = indx; i >= 0; i = P[i])
    {
        prevRot = prevRot * joints[i]->GetRotation();
    }

    return prevRot;
}

void SandBox::set_destination_pos(Eigen::Vector3d dir) {

    destination_position = tip_position + dir;

}

void SandBox::set_tip_pos() {
    tip_position = (CalcParentsTransJoint(num_of_links) * joints[num_of_joints - 1]->MakeTransd() * Eigen::Vector4d(0, 0, 0.05, 1)).head(3);
}

bool SandBox::isTooFar() {
    /*Vector3d FirstLink = (data_list[1].MakeTransd() * Vector4d(0, 0, -(link_Len/2),1)).head(3);
    double distance = (destination_position - FirstLink).norm();
    if (distance > links_number * link_Len)
        return true;
    return false;*/
    return true;
}
void SandBox::Skinning()
{
    if (recompute)
    {
        //find pose interval
        const int begin = (int)floor(anim_t) % poses.size();
        const int end = (int)(floor(anim_t) + 1) % poses.size();
        const double t = anim_t - floor(anim_t);

        // Interpolate pose and identity
        RotationList anim_pose(poses[begin].size());
        for (int e = 0; e < poses[begin].size(); e++)
        {
            anim_pose[e] = poses[begin][e].slerp(t, poses[end][e]);
        }

        //forwardLoop();
        //computeRotationTranslationJoints();
        igl::forward_kinematics(C, BE, P.head(16), anim_pose, vQ, vT);

        const int dim = C.cols();
        MatrixXd T(BE.rows() * (dim + 1), dim);
        for (int e = 0; e < BE.rows(); e++)
        {
            Affine3d a = Affine3d::Identity();
            a.translate(vT[e]);
            a.rotate(vQ[e]);
            T.block(e * (dim + 1), 0, dim + 1, dim) = a.matrix().transpose().block(0, 0, dim + 1, dim);
        }

        igl::dqs(data_list[0].V, W, vQ, vT, U);
        /*for (int i = 0; i < U.rows(); i++) {
            cout << " " << U.row(i) << end;
        }*/

        MatrixXd CT;
        MatrixXi BET;
        CT.resize(2*BE.rows() + 1, C.cols());
        BET.resize(BE.rows(), 2);
        for (int e = 0; e < BE.rows(); e++)
        {
            BET(e, 0) = 2*e;
            BET(e, 1) = 2*e + 1;
            Matrix4d t;
            t << T.block(e * 4, 0, 4, 3).transpose(), 0, 0, 0, 0;
            Affine3d a;
            a.matrix() = t;
            Vector3d c0 = C.row(BE(e, 0));
            Vector3d c1 = C.row(BE(e, 1));
            CT.row(2*e) = a * c0;
            CT.row(2*e + 1) = a * c1;
        }

        data_list[0].set_vertices(U);
        data_list[0].set_edges(CT, BET, sea_green);
        data_list[0].compute_normals();
        data_list[0].set_points(CT, RowVector3d(0, 0, 1));
        //isActive = false;
        //drawJoints();

        //if (isActive)
        //{
            anim_t += anim_t_dir;
        //}
        //else
        //{
          //  recompute = false;
        //}*/
    }
}

void SandBox::moveJoints()
{
    //bool isNotReach = true;
    //while (isNotReach) {
        //Vector3d destination = tip_position + Vector3d(4, 0, 0);
        for (int i = num_of_joints - 1; i > -1; i = P[i]) {
            Vector3d currJoint = getJoint(i);
            //cout << " currJoint" << i << ": " << currJoint << endl;
            Vector3d vec1 = (destination_position - currJoint);
            Vector3d vec2 = (tip_position - currJoint);
            Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(vec1, vec2);
            //Eigen::Matrix4d mat = CalcParentsTransJoint(i);
            Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
            for (int j = i; j > -1; j--) {
                mat = mat * joints[i]->GetRotation();
            }
            quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
            //vQ[i] = quat;
            joints[i]->RotateInSystem(mat.block<3, 3>(0, 0), quat);
            C.row(i) = getJoint(i);
            //cout << i << " pos: " << C.row(i) << endl;
            set_tip_pos();
            drawJoints();
            /*double distance = (tip_position - destination_position).norm();
            if (distance <= 0.1) {
                isNotReach = false;
            }*/
        }
    //}
    
}

//void SandBox::IK_solver(){
//
//    
//    set_tip_pos();
//    //Vector3d destination = JointsPoses[0] + Vector3d(4, 0, 0);
//    for (int i = links_number; i > 0; i = parents[i]) {
//        Vector3d currJoint = getJoint(i);
//        //cout << " currJoint" << i << ": " << currJoint << endl;
//        Vector3d vec1 = (destination_position - currJoint);
//        Vector3d vec2 = (tip_position - currJoint);
//        Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(vec1, vec2);
//        Eigen::Matrix4d mat = CalcParentsTrans(i);
//        quat = quat.slerp(0.95, Eigen::Quaterniond::Identity());
//        //vQ[i] = quat;
//        data_list[i].RotateInSystem(mat.block<3, 3>(0, 0), quat);
//        set_tip_pos();
//        
//    }
//    /*if (isTooFar())
//    {
//        cout << "cannot reach" << endl;
//        return;
//    }
//    for (size_t i = links_number; i > 0; i--) {
//        
//        Vector3d E = tip_position;
//        Vector3d D = destination_position;
//        Vector3d R = getJoint(i);
//        Vector3d RE = (E - R).normalized();
//        Vector3d RD = (D - R).normalized();
//
//        double distance = (D - E).norm();
//        if (distance < 0.1) {
//            isActive = false;
//            setEulerAngles();
//            std::cout << "distance " << distance << endl;
//            return;
//        }
//        
//        double cos_angle = RE.dot(RD);
//        if (cos_angle < -1) {
//            cos_angle = -1;
//        }
//        else if (cos_angle > 1) {
//            cos_angle = 1;
//        }
//        
//        Eigen::Vector3d vecToRotate = RE.cross(RD);
//        double angle = acos(cos_angle)/10;
//
//        data_list[i].RotateInSystem(vecToRotate, angle);
//        set_tip_pos();
//    }*/
//
//}


//void SandBox::setPositions() {
//    set_tip_pos();
//
//    for (int i = 1; i <= links_number; i++) {
//       positions[i -1] = getJoint(i);
//    }
//    startPos = positions[0];
//    positions[links_number] = tip_position;
//}

//from linknum to 1
//void SandBox::forward() {
//    tmpPositions[links_number] = destination_position;
//    for (signed int i = links_number - 1; i >= 0; i--){
//            double ri = (tmpPositions[i + 1] - tmpPositions[i]).norm();
//            double lambda = 1.6 / ri;
//            Vector3d newPos = (((1-lambda) * tmpPositions[i + 1]) + (lambda * tmpPositions[i])).normalized();
//            tmpPositions[i] = newPos;
//    }
//}

//from 1 to linknum
//void SandBox::backward() {
//    tmpPositions[0] = startPos;
//    for (size_t i = 0; i < links_number; i++) {
//        double ri = (tmpPositions[i + 1] - tmpPositions[i]).norm();
//        double lambda = 1.6 / ri;
//        Vector3d newPos = (((1 - lambda) * tmpPositions[i]) + (lambda * tmpPositions[i+1])).normalized();
//
//        tmpPositions[i+1] = newPos;
//    }
//}

//void SandBox::RotateJoints() {
//    for (size_t i = 0; i < links_number; i++) {
//
//        Vector3d oldVector = (positions[i +1] - positions[i]).normalized();
//        Vector3d newVector = (tmpPositions[i + 1] - tmpPositions[i]).normalized();
//
//        double cos_angle = newVector.dot(oldVector);
//        if (cos_angle < -1) {
//            cos_angle = -1;
//        }
//        else if (cos_angle > 1) {
//            cos_angle = 1;
//        }
//        Eigen::Vector3d vecToRotate = newVector.cross(oldVector);
//        double angle = acos(cos_angle) / 10;
//
//        data_list[i+1].MyRotate((CalcParentsRot(i+1) * data_list[i+1].GetRotation()).inverse() * (-vecToRotate), angle);
//        setPositions();
//    }
//}

//void SandBox::fabrik() {
//    
//    double distance = (tip_position - destination_position).norm();
//    if (isTooFar())
//    {
//        cout << "cannot reach" << endl;
//        isActive = false;
//        return;
//    }
//    if (distance > 0.1)
//    {
//        tmpPositions = positions;
//        forward();
//       backward();
//        RotateJoints();
//    }
//    else {
//        cout << "distance: " << distance << endl;
//        isActive = false;
//        return;
//    }
//   
//}




void SandBox::initDataStructure(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
   
    W.resize(data_list[0].V.rows(), joints.size() - 1);
    add_weights();
    /*Eigen::MatrixXd S;
    igl::jet(W, true, S);
    data_list[0].set_colors(S);*/
    data_list[0].tree.init(data_list[0].V, data_list[0].F);
    drawBox(&data_list[0].tree.m_box, 0);

    RotationList rest_pose;
    igl::directed_edge_orientations(C, BE, rest_pose);

    poses.resize(4, RotationList(16, Eigen::Quaterniond::Identity()));
    const Quaterniond twist(AngleAxisd(M_PI, Vector3d(1, 0, 0)));
    const Quaterniond bend(AngleAxisd(M_PI * 0.7, Vector3d(0, 0, 1)));
    for (int i = 1; i < num_of_links; i++) {
        poses[1][i] = rest_pose[2] * twist * rest_pose[2].conjugate();
        poses[3][i] = rest_pose[2] * bend * rest_pose[2].conjugate();
    }
    
}


bool SandBox::checkConditions(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree, size_t i) {

    AlignedBox<double, 3> boxA = Atree->m_box;
    AlignedBox<double, 3> boxB = Btree->m_box;

    size_t currData = 0;

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
            //subTrees[selected_data_index] = Atree;
            //subTrees[i] = Btree;
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
    //velocities[selected_data_index] = dir;
}

void SandBox::Animate()
{
    
    if (isActive)
    {
        for (size_t i = 1; i < data_list.size(); i++) {
            //if (isTooFar(tip_position)){
            if (collisionDetec(&data_list[0].tree, &data_list[i].tree, i)) {
                cout << "collision!!" << endl;
                if (!data_list[i].isPrize) {
                    isActive = false;
                    resetScene(false);
                }
                else {
                    score += data_list[i].gamePoints;
                    erase_mesh(i);
                }
                if (score == 10) { 
                    isActive = false;
                    resetScene(true);
                };
            }
        }
        time_t currentTime;
        if (difftime(time(&currentTime), level_start_time) >= 240) {
            cout << "times up" << endl;
            isActive = false;
            resetScene(false);
        }
    }
}