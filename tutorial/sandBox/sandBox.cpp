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

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
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



void SandBox::initDataStructure(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	Eigen::MatrixXi* Etmp = new Eigen::MatrixXi(), * EFtmp = new Eigen::MatrixXi(), * EItmp = new Eigen::MatrixXi();
	Eigen::VectorXi* EMAPtmp = new Eigen::VectorXi();
	SandBox::PriorityQueue* Qtmp = new SandBox::PriorityQueue;
	vector<SandBox::PriorityQueue::iterator> QitTmp;

	edge_flaps(F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);

	QitTmp.resize(Etmp->rows());
	C.push_back(new Eigen::MatrixXd(Etmp->rows(), V.cols()));
	Eigen::VectorXd costs(Etmp->rows());
	Qtmp->clear();


	for (int e = 0; e < Etmp->rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
        shortest_edge_and_midpoint(e, V, F, *Etmp, *EMAPtmp, *EFtmp, *EItmp, cost, p);
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

    /*delete EMAPtmp;
    delete Etmp;
    delete EFtmp;
    delete EItmp;
    delete Qtmp;*/

    
 }


void SandBox::simplification()
{
    if (!Q[selected_data_index]->empty())
    {

        bool something_collapsed = false;
        int numToCollapse = std::ceil(0.05 * Q[selected_data_index]->size());

        for (int j = 0; j < numToCollapse; j++)
        {

            if (!collapse_edge(shortest_edge_and_midpoint, data().V, data().F,
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