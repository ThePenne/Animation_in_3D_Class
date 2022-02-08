#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	std::vector<igl::AABB<Eigen::MatrixXd, 3>*> trees;
	std::vector<igl::AABB<Eigen::MatrixXd, 3>*> sub_trees;
	SandBox();

	void check_and_handle_intersect(int obj);
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
private:
	// Prepare array-based edge data structures and priority queue


	void Animate();
};

