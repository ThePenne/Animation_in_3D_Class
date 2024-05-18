#include "../igl_inline.h"
#include "MeshGL.h"
#include <cassert>
#include <cstdint>
#include "Movable.h"
//#include <Eigen/Core>
#include <memory>
#include <vector>

namespace igl
{

// TODO: write documentation
namespace opengl
{
class Joint : public Movable
{
public:
Joint();
Eigen::Vector3d pos;
int parent;
};
} // namespace opengl
} // namespace igl