#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	void print_transformations();
	void print_tip_positions();
	void print_destination();
	void CCD_iteration();
	void FABRIK_iteration();
	~SandBox();
	void Init(const std::string& config);
	double doubleVariable;
	bool FABRIK;
private:
	// Prepare array-based edge data structures and priority queue
	
	
	void Animate();
};

