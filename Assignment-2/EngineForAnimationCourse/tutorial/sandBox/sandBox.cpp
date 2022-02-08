#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "igl/opengl/glfw/Renderer.h"
#include "Eigen/dense"
#include <functional>


void AddBox(igl::opengl::ViewerData& data, const Eigen::AlignedBox<double, 3>& box, const Eigen::RowVector3d& color);
bool boxes_intersect(Eigen::AlignedBox<double, 3> A, Eigen::AlignedBox<double, 3> B, Eigen::Matrix4d Atrans, Eigen::Matrix4d Btrans, Eigen::Matrix3d Arot, Eigen::Matrix3d Brot);
Eigen::Vector3d transform_vec(const Eigen::Matrix4d& trans, Eigen::Vector3d vec3);
bool recursive_intersects(igl::AABB<Eigen::MatrixXd, 3>* tree1, Eigen::Matrix4d trans1, Eigen::Matrix3d rot1, igl::opengl::ViewerData& data1, igl::AABB<Eigen::MatrixXd, 3>* tree2, Eigen::Matrix4d trans2, Eigen::Matrix3d rot2, igl::opengl::ViewerData& data2);

SandBox::SandBox() : trees(10), sub_trees(10) {}

void SandBox::check_and_handle_intersect(int obj)
{
	if (data_vel[obj] == igl::opengl::glfw::none)
		return;
	igl::AABB<Eigen::MatrixXd, 3>* obj_tree;
	Eigen::Matrix4d obj_trans;
	Eigen::Matrix3d obj_rot;
	Eigen::AlignedBox<double, 3>obj_tree_left_box;
	Eigen::AlignedBox<double, 3>obj_tree_right_box;

	igl::AABB<Eigen::MatrixXd, 3>* other_tree;
	Eigen::Matrix4d other_trans;
	Eigen::Matrix3d other_rot;
	Eigen::AlignedBox<double, 3>other_tree_left_box;
	Eigen::AlignedBox<double, 3>other_tree_right_box;
	Eigen::Vector3d collision_color(1, 1, 1);

	for (int i = 0; i < data_list.size(); i++)
	{
		if (i == obj)
			continue;

		obj_tree = trees[obj];
		obj_trans = data_list[obj].MakeTransScaled();
		obj_rot = data_list[obj].GetRotation();
		
		other_tree = trees[i];
		other_trans = data_list[i].MakeTransScaled();
		other_rot = data_list[i].GetRotation();
		
		if (recursive_intersects(obj_tree, obj_trans, obj_rot, data_list[obj], other_tree, other_trans, other_rot, data_list[i]))
		{
			data_vel[obj] = igl::opengl::glfw::none;
		}
	}
}

bool recursive_intersects(igl::AABB<Eigen::MatrixXd, 3>* tree1, Eigen::Matrix4d trans1, Eigen::Matrix3d rot1, igl::opengl::ViewerData& data1, igl::AABB<Eigen::MatrixXd, 3>* tree2, Eigen::Matrix4d trans2, Eigen::Matrix3d rot2, igl::opengl::ViewerData& data2)
{
	if(boxes_intersect(tree1->m_box, tree2->m_box, trans1, trans2, rot1, rot2))
	{
		if(tree1->is_leaf() && tree2->is_leaf())
		{
			AddBox(data1, tree1->m_box, Eigen::Vector3d(1, 1, 1));
			AddBox(data2, tree2->m_box, Eigen::Vector3d(1, 1, 1));
			return true;
		}

		igl::AABB<Eigen::MatrixXd, 3>* tree1_left = tree1;
		igl::AABB<Eigen::MatrixXd, 3>* tree1_right = tree1;

		igl::AABB<Eigen::MatrixXd, 3>* tree2_left = tree2;
		igl::AABB<Eigen::MatrixXd, 3>* tree2_right = tree2;

		if(!tree1->is_leaf())
		{
			tree1_left = tree1->m_left;
			tree1_right = tree1->m_right;
		}
		if (!tree2->is_leaf())
		{
			tree2_left = tree2->m_left;
			tree2_right = tree2->m_right;
		}

		return	recursive_intersects(tree1_left, trans1, rot1, data1, tree2_left, trans2, rot2, data2) ||
				recursive_intersects(tree1_left, trans1, rot1, data1, tree2_right, trans2, rot2, data2) ||
				recursive_intersects(tree1_right, trans1, rot1, data1, tree2_left, trans2, rot2, data2) ||
				recursive_intersects(tree1_right, trans1, rot1, data1, tree2_right, trans2, rot2, data2);
	}
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
		int obj_count = 0;

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

			// ass 2
			trees[obj_count] = new igl::AABB<Eigen::MatrixXd, 3>();
			trees[obj_count]->init(data().V, data().F);
			AddBox(data(), trees[obj_count]->m_box, Eigen::Vector3d(0, 1, 0));
			data().TranslateInSystem(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.5 * obj_count, 1 * obj_count, 0));
			obj_count++;
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

void AddBox(igl::opengl::ViewerData& data, const Eigen::AlignedBox<double, 3>& box, const Eigen::RowVector3d& color)
{
	/*
	Box vertices arrange like this
			v5-------v6
		   /|       /|
		  / |      / |
		 v4-------v7 |
		 |  |     |  |
		 |  v1----|-v2
		 | /      | /
		 |/       |/
		 v0-------v3
	*/

	// Floor vertices
	Eigen::Vector3d v0 = box.corner(Eigen::AlignedBox<double, 3>::BottomLeftFloor);
	Eigen::Vector3d v1 = box.corner(Eigen::AlignedBox<double, 3>::TopLeftFloor);
	Eigen::Vector3d v2 = box.corner(Eigen::AlignedBox<double, 3>::TopRightFloor);
	Eigen::Vector3d v3 = box.corner(Eigen::AlignedBox<double, 3>::BottomRightFloor);

	// Ceil vertices
	Eigen::Vector3d v4 = box.corner(Eigen::AlignedBox<double, 3>::BottomLeftCeil);
	Eigen::Vector3d v5 = box.corner(Eigen::AlignedBox<double, 3>::TopLeftCeil);
	Eigen::Vector3d v6 = box.corner(Eigen::AlignedBox<double, 3>::TopRightCeil);
	Eigen::Vector3d v7 = box.corner(Eigen::AlignedBox<double, 3>::BottomRightCeil);

	Eigen::MatrixXd P1;
	Eigen::MatrixXd P2;
	Eigen::MatrixXd C;
	P1.resize(12, 3);
	P2.resize(12, 3);
	C.resize(12, 3);

	// Edge colors
	for (int i = 0; i < 12; i++) C.row(i) = color;

	// Floor edges
	P1.row(0) = v0; P2.row(0) = v1;
	P1.row(1) = v1; P2.row(1) = v2;
	P1.row(2) = v2; P2.row(2) = v3;
	P1.row(3) = v3; P2.row(3) = v0;

	// Ceil edges
	P1.row(4) = v4; P2.row(4) = v5;
	P1.row(5) = v5; P2.row(5) = v6;
	P1.row(6) = v6; P2.row(6) = v7;
	P1.row(7) = v7; P2.row(7) = v4;

	// Floor-Ceil linking edges
	P1.row(8) = v0; P2.row(8) = v4;
	P1.row(9) = v1; P2.row(9) = v5;
	P1.row(10) = v2; P2.row(10) = v6;
	P1.row(11) = v3; P2.row(11) = v7;

	data.add_edges(P1, P2, C);
}

bool boxes_intersect(Eigen::AlignedBox<double, 3> A, Eigen::AlignedBox<double, 3> B, Eigen::Matrix4d Atrans, Eigen::Matrix4d Btrans, Eigen::Matrix3d Arot, Eigen::Matrix3d Brot)
{
	Eigen::Vector3d Pa = transform_vec(Atrans, A.center());
	Eigen::Vector3d Ax = Arot * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d Ay = Arot * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d Az = Arot * Eigen::Vector3d(0, 0, 1);
	double Wa = A.sizes()[0] / 2;
	double Ha = A.sizes()[1] / 2;
	double Da = A.sizes()[2] / 2;

	Eigen::Vector3d Pb = transform_vec(Btrans, B.center());
	Eigen::Vector3d Bx = Brot * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d By = Brot * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d Bz = Brot * Eigen::Vector3d(0, 0, 1);
	double Wb = B.sizes()[0] / 2;
	double Hb = B.sizes()[1] / 2;
	double Db = B.sizes()[2] / 2;

	Eigen::Vector3d T = Pb - Pa;
	Eigen::Matrix3d R;
	// Rij = Ai.dot(Bj)
	R(0, 0) = Ax.dot(Bx);
	R(0, 1) = Ax.dot(By);
	R(0, 2) = Ax.dot(Bz);
	R(1, 0) = Ay.dot(Bx);
	R(1, 1) = Ay.dot(By);
	R(1, 2) = Ay.dot(Bz);
	R(2, 0) = Az.dot(Bx);
	R(2, 1) = Az.dot(By);
	R(2, 2) = Az.dot(Bz);

	// CASE 1:
	// L = Ax
	// If true, L is a separating axis parallel to Ax (and there exists a separating plane with normal, Ax)
	if (abs(T.dot(Ax)) > Wa + abs(Wb * R(0, 0)) + abs(Hb * R(0, 1)) + abs(Db * R(0, 2)))
		return false;
	// CASE 2:
	// L = Ay
	// If true, L is a separating axis parallel to Ay (and there exists a separating plane with normal, Ay)
	if (abs(T.dot(Ay)) > Ha + abs(Wb * R(1, 0)) + abs(Hb * R(1, 1)) + abs(Db * R(1, 2)))
		return false;
	// CASE 3:
	// L = Az
	// If true, L is a separating axis parallel to Az (and there exists a separating plane with normal, Az)
	if (abs(T.dot(Az)) > Da + abs(Wb * R(2, 0)) + abs(Hb * R(2, 1)) + abs(Db * R(2, 2)))
		return false;
	// CASE 4:
	// L = Bx
	// If true, L is a separating axis parallel to Bx (and there exists a separating plane with normal, Bx)
	if (abs(T.dot(Bx)) > Wb + abs(Wa * R(0, 0)) + abs(Ha * R(1, 0)) + abs(Da * R(2, 0)))
		return false;
	// CASE 5:
	// L = By
	// If true, L is a separating axis parallel to By (and there exists a separating plane with normal, By)
	if (abs(T.dot(By)) > Hb + abs(Wa * R(0, 1)) + abs(Ha * R(1, 1)) + abs(Da * R(2, 1)))
		return false;
	// CASE 6:
	// L = Bz
	// If true, L is a separating axis parallel to Bz (and there exists a separating plane with normal, Bz)
	if (abs(T.dot(Bz)) > Db + abs(Wa * R(0, 2)) + abs(Ha * R(1, 2)) + abs(Da * R(2, 2)))
		return false;
	// CASE 7:
	// L = Ax * Bx
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ax and Bx
	if (abs(T.dot(Az) * R(1, 0) - T.dot(Ay) * R(2, 0)) > abs(Ha * R(2, 0)) + abs(Da * R(1, 0)) + abs(Hb * R(0, 2)) + abs(Db * R(0, 1)))
		return false;
	// CASE 8 :
	// L = Ax * By
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ax and By
	if (abs(T.dot(Az) * R(1, 1) - T.dot(Ay) * R(2, 1)) > abs(Ha * R(2, 1)) + abs(Da * R(1, 1)) + abs(Wb * R(0, 2)) + abs(Db * R(0, 0)))
		return false;
	// CASE 9 
	// L = Ax * Bz
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ax and Bz
	if (abs(T.dot(Az) * R(1, 2) - T.dot(Ay) * R(2, 2)) > abs(Ha * R(2, 2)) + abs(Da * R(1, 2)) + abs(Wb * R(0, 1)) + abs(Hb * R(0, 0)))
		return false;
	// CASE 10:
	// L = Ay * Bx
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ay and Bx
	if (abs(T.dot(Ax) * R(2, 0) - T.dot(Az) * R(0, 0)) > abs(Wa * R(2, 0)) + abs(Da * R(0, 0)) + abs(Hb * R(1, 2)) + abs(Db * R(1, 1)))
		return false;
	// CASE 11 :
	// L = Ay * By
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ay and By
	if (abs(T.dot(Ax) * R(2, 1) - T.dot(Az) * R(0, 1)) > abs(Wa * R(2, 1)) + abs(Da * R(0, 1)) + abs(Wb * R(1, 2)) + abs(Db * R(1, 0)))
		return false;
	// CASE 12 :
	// L = Ay * Bz
	// If true, L is a separating axis perpendicular to the separating plane spanned by Ay and Bz
	if (abs(T.dot(Ax) * R(2, 2) - T.dot(Az) * R(0, 2)) > abs(Wa * R(2, 2)) + abs(Da * R(0, 2)) + abs(Wb * R(1, 1)) + abs(Hb * R(1, 0)))
		return false;
	// CASE 13 :
	// L = Az * Bx
	// If true, L is a separating axis perpendicular to the separating plane spanned by Az and Bx
	if (abs(T.dot(Ay) * R(0, 0) - T.dot(Ax) * R(1, 0)) > abs(Wa * R(1, 0)) + abs(Ha * R(0, 0)) + abs(Hb * R(2, 2)) + abs(Db * R(2, 1)))
		return false;
	// CASE 14 :
	// L = Az * By
	// If true, L is a separating axis perpendicular to the separating plane spanned by Az and By
	if (abs(T.dot(Ay) * R(0, 1) - T.dot(Ax) * R(1, 1)) > abs(Wa * R(1, 1)) + abs(Ha * R(0, 1)) + abs(Wb * R(2, 2)) + abs(Db * R(2, 0)))
		return false;
	// CASE 15 :
	// L = Az * Bz
	// If true, L is a separating axis perpendicular to the separating plane spanned by Az and Bz
	if (abs(T.dot(Ay) * R(0, 2) - T.dot(Ax) * R(1, 2)) > abs(Wa * R(1, 2)) + abs(Ha * R(0, 2)) + abs(Wb * R(2, 1)) + abs(Hb * R(2, 0)))
		return false;

	return true;
}

Eigen::Vector3d transform_vec(const Eigen::Matrix4d& trans, Eigen::Vector3d vec3)
{
	Eigen::Vector4d vec4(vec3.x(), vec3.y(), vec3.z(), 1);

	vec4 = trans * vec4;
	vec3 << vec4.x(), vec4.y(), vec4.z();
	return vec3;
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{



	}
}


