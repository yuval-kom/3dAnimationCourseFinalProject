#include "joint.h"

using namespace Eigen;
namespace igl
{
	namespace opengl
	{
		Joint::Joint(Vector3d pos, int _id) {
				position = pos;
				id = _id;
				//V = pos;
			}
		Joint::~Joint(){}
			
	}
}

