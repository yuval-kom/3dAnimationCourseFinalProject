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

IK::IK()
{


}

IK::~IK()
{
    

}



void IK::setInitialPosition() {

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

void IK::drawAxis(size_t i){
    //maybe change
    data_list[i].add_edges(Eigen::RowVector3d(-1.6, 0, 0.8), Eigen::RowVector3d(1.6, 0, 0.8), Eigen::RowVector3d(1, 0, 0)); // x axis - red
    data_list[i].add_edges(Eigen::RowVector3d(0, -1.6, 0.8), Eigen::RowVector3d(0, 1.6, 0.8), Eigen::RowVector3d(0, 1, 0)); // y axis - green
    data_list[i].add_edges(Eigen::RowVector3d(0, 0,-0.8), Eigen::RowVector3d(0, 0, 2.4), Eigen::RowVector3d(0, 0, 1)); // z axis - blue
}

void IK::set_destination_pos() {
    
    destination_position = (data_list[0].MakeTransd() * Vector4d(0,0, 0, 1)).head(3);
    //destination_position = data_list[0].GetRotation() * Vector3d(0, 0, 0);

}

void IK::set_tip_pos() {
    
   /* Vector3d tipTmp = (data_list[links_number].MakeTransd() * Vector4d(0, 0, 0, 1)).head(3) + data_list[links_number].GetRotation()* Vector3d(0, 0, link_Len / 2);
    Matrix3d Rtmp = data_list[links_number].GetRotation();
    for (size_t j = links_number; j > 0; j--) {
        Rtmp = Rtmp * data_list[j].GetRotation();
    }
    Vector3d tmp = Rtmp * Vector3d(0, 0, link_Len);
    tipTmp = tipTmp + tmp;
    tip_position = tipTmp;*/
    tip_position = (CalcParentsTrans(links_number) * data_list[links_number].MakeTransd() * Vector4d(0, 0, link_Len/2,1)).head(3);
    //tip_position = (data_list[links_number].MakeTransd() * Vector4d(0, 0, -(link_Len/2),1)).head(3);
    //cout << "tip pos: " << tip_position << endl;
}

bool IK::isTooFar() {
    Vector3d FirstLink = (data_list[1].MakeTransd() * Vector4d(0, 0, (link_Len/2),1)).head(3);
    double distance = (destination_position - FirstLink).norm();
    if (distance > links_number * link_Len)
        return true;
    return false;
}

void IK::ccd(){

    set_destination_pos();
    set_tip_pos();
    Vector3d O = (data_list[links_number].MakeTransd() * Vector4d(0, 0, 0, 1)).head(3);
    if (isTooFar())
    {
        cout << "cannot reach" << endl;
        return;
    }
    for (size_t i = links_number; i > 0; i--) {
        
        Vector3d E = tip_position;
        Vector3d D = destination_position;
        Vector3d R = O + data_list[links_number].GetRotation() * Vector3d(0, 0, -link_Len / 2);
        Matrix3d Rtmp = data_list[links_number].GetRotation();
        for (size_t j = links_number; j > i; j--) {
            Rtmp = Rtmp * data_list[j].GetRotation();
        }
        Vector3d tmp = Rtmp * Vector3d(0, 0, -link_Len);
        if(i!= links_number)
            R = R + tmp;
        Vector3d RE = (E - R).normalized();
        Vector3d RD = (D - R).normalized();

        double distance = (D - E).norm();
        std::cout << "distance2 " << distance << endl;
        if (distance < 0.1) {
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
        
        Eigen::Vector3d rotationAxis = RE.cross(RD);
        double angle = acos(cos_angle)/10;
        data_list[i].RotateInSystem(rotationAxis, angle);
        set_tip_pos();
    }

}

void IK::printEulerAngles(){
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


void IK::Animate()
{
    //Assignment2
    //SetAnimation();
    if (isActive)
    {

    }
}