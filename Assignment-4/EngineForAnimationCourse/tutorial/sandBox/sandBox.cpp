#include "tutorial/sandBox/sandBox.h"
#include "Eigen/dense"
#include <functional>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readOBJ.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/boundary_loop.h>
#include <igl/map_vertices_to_circle.h>
#include <igl/harmonic.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <random>
#include <ctime>

void calc_w(const Eigen::MatrixXd &V, const Eigen::MatrixXd &C, Eigen::MatrixXd &W);
Eigen::Vector3d transform_vec3(Eigen::Matrix4d trans, Eigen::Vector3d vec);

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
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		int idx = 0;
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);

			if (idx == 0)
			{
				parents.push_back(-1);
				setup_snake(item_name);
				C_colors.resizeLike(C);
				for (int i = 0; i < C.rows(); i++)
					C_colors.row(i) << 0, 0, 1;
				E_colors.resize(BE.rows(), 3);
				for (int i = 0; i < BE.rows(); i++)
					E_colors.row(i) << 70. / 255., 252. / 255., 167. / 255.;
				data().add_points(C, C_colors);
				data().set_edges(C, BE, E_colors);
				data().show_overlay_depth = false;
				data().point_size = 5;
				data().line_width = 2;
			}
			else
			{
				parents.push_back(-1);
			}
			idx++;
		}
		snake_obj = 0;
		cubemap = 1;
		region_wall_front = 2;
		region_wall_back = 3;
		region_wall_left = 4;
		region_wall_right = 5;
		region_wall_top = 6;
		region_wall_bottom = 7;
		region_box = 8;
		apple_obj = 9;
		setup_region_box();
		setup_region_walls();
		data(apple_obj).MyScale(Eigen::Vector3d::Ones() * 2);
		data(apple_obj).set_colors(Eigen::RowVector3d(45. / 255., 130. / 255., 38. / 255.));
		data(snake_obj).set_colors(Eigen::RowVector3d(181. / 255., 131. / 255., 65. / 255.));
		data(cubemap).MyScale(Eigen::Vector3d::Ones() * 200);
		data(cubemap).cubemap_texture = igl::opengl::ViewerData::loadCubemap(faces[1]);
		data(cubemap).MyRotate(Eigen::Vector3d::UnitX(), -igl::PI / 2);
		nameFileout.close();
		selected_data_index = 0;
	}
}

void SandBox::setup_region_box()
{
	data(region_box).MyScale(Eigen::Vector3d(42, 42, 42));
	data(region_box).show_overlay |= 1;
	trees.emplace_back(new igl::AABB<Eigen::MatrixXd, 3>());
	trees.back()->init(data(region_box).V, data(region_box).F);
	add_box(data(region_box), trees.back()->m_box, Eigen::Vector3d::Ones());
	region_lines = data(region_box).lines;
}

void SandBox::setup_region_walls()
{
	data(region_wall_front).MyScale(Eigen::Vector3d(42, 0.001, 42));
	data(region_wall_front).MyTranslate(Eigen::Vector3d(0, -21, 0), true);
	data(region_wall_front).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);

	data(region_wall_back).MyScale(Eigen::Vector3d(42, 0.001, 42));
	data(region_wall_back).MyTranslate(Eigen::Vector3d(0, 21, 0), true);
	data(region_wall_back).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);

	data(region_wall_left).MyScale(Eigen::Vector3d(0.001, 42, 42));
	data(region_wall_left).MyTranslate(Eigen::Vector3d(21, 0, 0), true);
	data(region_wall_left).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);

	data(region_wall_right).MyScale(Eigen::Vector3d(0.001, 42, 42));
	data(region_wall_right).MyTranslate(Eigen::Vector3d(-21, 0, 0), true);
	data(region_wall_right).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);

	data(region_wall_top).MyScale(Eigen::Vector3d(42, 42, 0.001));
	data(region_wall_top).MyTranslate(Eigen::Vector3d(0, 0, -21), true);
	data(region_wall_top).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);

	data(region_wall_bottom).MyScale(Eigen::Vector3d(42, 42, 0.001));
	data(region_wall_bottom).MyTranslate(Eigen::Vector3d(0, 0, 21), true);
	data(region_wall_bottom).set_colors(Eigen::RowVector4d(108, 213, 224, 100) / 255.);
}

void SandBox::CalcSnakeUV()
{
	Eigen::MatrixXd V_uv;
	Eigen::VectorXi bnd;
	Eigen::MatrixXi F = data().F;
	igl::boundary_loop(F, bnd);
	Eigen::MatrixXd bnd_uv;
	igl::map_vertices_to_circle(data().V, bnd, bnd_uv);

	igl::harmonic(data().V, data().F, bnd, bnd_uv, 1, V_uv);

	data().set_uv(V_uv);
}

SandBox::~SandBox()
{
	mciSendString("close music", NULL, 0, NULL);
}

void SandBox::Animate(igl::opengl::ViewerCore &camera)
{
	if (isActive)
	{
		RotateCamera(camera);
		set_walls();
		MoveSnake();
		MoveMines();
		check_and_handle_snake_intersect();
		if (anim_t > 1)
			anim_t = 0;
		else
			anim_t += anim_t_dif;
		if (time_passed % max_level_time == 0)
		{
			level_finished = true;
		}
		time_passed++;
	}
	if (start_music)
	{
		StartMusic();
		start_music = false;
	}
	if (stop_music)
	{
		StopMusic();
		stop_music = false;
	}
}

void SandBox::MoveMines()
{
	Eigen::Vector3d movement;
	for (auto &mine : mines)
	{
		movement = Eigen::Vector3d::Zero();
		movement(mine[1]) = mines_speed * mine[2];
		data(mine[0]).MyTranslate(movement, true);
		if (std::abs(data(mine[0]).MakeTransd().col(3).head(3)(mine[1])) > 21)
		{
			mine[2] = -mine[2];
			data(mine[0]).MyTranslate(-movement, true);
		}
	}
}

void SandBox::UpdateNextPose()
{
	Eigen::Vector3d axis;
	double angle = 0;

	switch (_arrow_dir)
	{
	case ARROW_LEFT:
		UpdateSnakeLeft(axis);
		break;
	case ARROW_RIGHT:
		UpdateSnakeRight(axis);
		break;
	case ARROW_UP:
		UpdateSnakeUp(axis);
		break;
	case ARROW_DOWN:
		UpdateSnakeDown(axis);
		break;
	case ARROW_NONE:
		return;
	}
	direction_changed = true;

	Eigen::Quaterniond turn(Eigen::AngleAxisd(igl::PI / 2, curr_pose[0].toRotationMatrix().transpose() * axis));
	next_pose[0] = curr_pose[0] * turn;
	Eigen::Quaterniond turn2(Eigen::AngleAxisd(-igl::PI / 2, curr_pose[0].toRotationMatrix().transpose() * axis));
	next_pose[1] = turn2;
	direction_change = true;

	_arrow_dir = ARROW_NONE;
}

void SandBox::UpdateSnakeLeft(Eigen::Vector3d &axis)
{
	switch (_axes_sys)
	{
	case FRONT:
		axis << 0, -1, 0;
		break;
	case BACK:
		axis << 0, 1, 0;
		break;
	case TOP:
		axis << 0, 0, -1;
		break;
	case BOTTOM:
		axis << 0, 0, 1;
		break;
	}
	switch (_snake_dir)
	{
	case SNAKE_LEFT:
		_snake_dir = SNAKE_DOWN;
		break;
	case SNAKE_RIGHT:
		_snake_dir = SNAKE_UP;
		break;
	case SNAKE_UP:
		_snake_dir = SNAKE_LEFT;
		break;
	case SNAKE_DOWN:
		_snake_dir = SNAKE_RIGHT;
		break;
	}
}

void SandBox::UpdateSnakeRight(Eigen::Vector3d &axis)
{
	switch (_axes_sys)
	{
	case FRONT:
		axis << 0, 1, 0;
		break;
	case BACK:
		axis << 0, -1, 0;
		break;
	case TOP:
		axis << 0, 0, 1;
		break;
	case BOTTOM:
		axis << 0, 0, -1;
		break;
	}
	switch (_snake_dir)
	{
	case SNAKE_LEFT:
		_snake_dir = SNAKE_UP;
		break;
	case SNAKE_RIGHT:
		_snake_dir = SNAKE_DOWN;
		break;
	case SNAKE_UP:
		_snake_dir = SNAKE_RIGHT;
		break;
	case SNAKE_DOWN:
		_snake_dir = SNAKE_LEFT;
		break;
	}
}

void SandBox::UpdateSnakeUp(Eigen::Vector3d &axis)
{
	switch (_axes_sys)
	{
	case FRONT:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 0, 1;
			break;
		case SNAKE_RIGHT:
			axis << 0, 0, -1;
			break;
		case SNAKE_UP:
			axis << 1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << -1, 0, 0;
			break;
		}
		_axes_sys = TOP;
		break;
	case BACK:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 0, -1;
			break;
		case SNAKE_RIGHT:
			axis << 0, 0, 1;
			break;
		case SNAKE_UP:
			axis << 1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << -1, 0, 0;
			break;
		}
		_axes_sys = BOTTOM;
		break;
	case TOP:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, -1, 0;
			break;
		case SNAKE_RIGHT:
			axis << 0, 1, 0;
			break;
		case SNAKE_UP:
			axis << 1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << -1, 0, 0;
			break;
		}
		_axes_sys = BACK;
		break;
	case BOTTOM:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 1, 0;
			break;
		case SNAKE_RIGHT:
			axis << 0, -1, 0;
			break;
		case SNAKE_UP:
			axis << 1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << -1, 0, 0;
			break;
		}
		_axes_sys = FRONT;
		break;
	}
	_snake_dir = SNAKE_UP;
}

void SandBox::UpdateSnakeDown(Eigen::Vector3d &axis)
{
	switch (_axes_sys)
	{
	case FRONT:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 0, -1;
			break;
		case SNAKE_RIGHT:
			axis << 0, 0, 1;
			break;
		case SNAKE_UP:
			axis << -1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << 1, 0, 0;
			break;
		}
		_axes_sys = BOTTOM;
		break;
	case BACK:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 0, 1;
			break;
		case SNAKE_RIGHT:
			axis << 0, 0, -1;
			break;
		case SNAKE_UP:
			axis << -1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << 1, 0, 0;
			break;
		}
		_axes_sys = TOP;
		break;
	case TOP:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, 1, 0;
			break;
		case SNAKE_RIGHT:
			axis << 0, -1, 0;
			break;
		case SNAKE_UP:
			axis << -1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << 1, 0, 0;
			break;
		}
		_axes_sys = FRONT;
		break;
	case BOTTOM:
		switch (_snake_dir)
		{
		case SNAKE_LEFT:
			axis << 0, -1, 0;
			break;
		case SNAKE_RIGHT:
			axis << 0, 1, 0;
			break;
		case SNAKE_UP:
			axis << -1, 0, 0;
			break;
		case SNAKE_DOWN:
			axis << 1, 0, 0;
			break;
		}
		_axes_sys = BACK;
		break;
	}
	_snake_dir = SNAKE_UP;
}

void SandBox::RotateCamera(igl::opengl::ViewerCore &camera)
{
	using namespace Eigen;
	if (anim_t > 1)
	{
		curr_camera_rotation_dir = next_camera_rotation_dir;
		next_camera_rotation_dir = 0;
	}
	else if (anim_t > 0)
	{
		double theta = curr_camera_rotation_dir * anim_t_dif * igl::PI / 2;
		Matrix3f rot = AngleAxisf(theta, Vector3f::UnitX()).matrix();
		camera.camera_eye = rot * camera.camera_eye;
		camera.camera_up = rot * camera.camera_up;
	}
}

void SandBox::set_walls()
{
	if (curr_camera_rotation_dir > 0 && anim_t == 0)
	{
		switch (_wall_sys)
		{
		case FRONT:
			data(region_wall_top).show_faces = 0;
			_wall_sys = TOP;
			break;
		case BACK:
			data(region_wall_bottom).show_faces = 0;
			_wall_sys = BOTTOM;
			break;
		case TOP:
			data(region_wall_back).show_faces = 0;
			_wall_sys = BACK;
			break;
		case BOTTOM:
			data(region_wall_front).show_faces = 0;
			_wall_sys = FRONT;
			break;
		}
	}
	else if (curr_camera_rotation_dir < 0 && anim_t == 0)
	{
		switch (_wall_sys)
		{
		case FRONT:
			data(region_wall_bottom).show_faces = 0;
			_wall_sys = BOTTOM;
			break;
		case BACK:
			data(region_wall_top).show_faces = 0;
			_wall_sys = TOP;
			break;
		case TOP:
			data(region_wall_front).show_faces = 0;
			_wall_sys = FRONT;
			break;
		case BOTTOM:
			data(region_wall_back).show_faces = 0;
			_wall_sys = BACK;
			break;
		}
	}
	else if (curr_camera_rotation_dir == 0)
	{
		data(region_wall_front).show_faces = _axes_sys == FRONT ? 0 : 1;
		data(region_wall_back).show_faces = _axes_sys == BACK ? 0 : 1;
		data(region_wall_top).show_faces = _axes_sys == TOP ? 0 : 1;
		data(region_wall_bottom).show_faces = _axes_sys == BOTTOM ? 0 : 1;
	}
}

Eigen::Vector3d transform_vec3(Eigen::Matrix4d trans, Eigen::Vector3d vec3)
{
	Eigen::Vector4d vec4(vec3.x(), vec3.y(), vec3.z(), 1);
	return (trans * vec4).head(3);
}

void SandBox::MoveSnake()
{
	BendSnake();

	Eigen::Affine3d movment_rot = Eigen::Affine3d::Identity();
	movment_rot.rotate(curr_pose[0]);
	Eigen::Vector3d movment_vec = movment_rot.matrix().block(0, 0, 3, 3) * Eigen::Vector3d(0, 0, -1);
	data(snake_obj).MyTranslate(movment_vec * (anim_t_dif + 0.05), true);

	if (anim_t > 1)
	{
		if (direction_changed)
			direction_changed = false;
		else
			UpdateNextPose();
		rest_pose = curr_pose;
		curr_pose = next_pose;
		for (int i = next_pose.size() - 1; i > 1; i--)
		{
			next_pose[i] = curr_pose[i - 1];
		}
		if (direction_change)
		{
			next_pose[1] = Eigen::Quaterniond::Identity();
			direction_change = false;
		}

		swivel_dir = !swivel_dir;
	}
}

void SandBox::BendSnake()
{

	using namespace Eigen;
	using namespace std;

	// Interpolate pose and identity
	RotationList anim_rest_pose(curr_pose.size());
	RotationList anim_pose(curr_pose.size());

	Eigen::Quaterniond swivel_right(Eigen::AngleAxisd(igl::PI / 8, Eigen::Vector3d(0, 1, 0)));
	Eigen::Quaterniond swivel_left(Eigen::AngleAxisd(-igl::PI / 8, Eigen::Vector3d(0, 1, 0)));
	Eigen::Quaterniond swivel2_right(Eigen::AngleAxisd(igl::PI / 4, Eigen::Vector3d(0, 1, 0)));
	Eigen::Quaterniond swivel2_left(Eigen::AngleAxisd(-igl::PI / 4, Eigen::Vector3d(0, 1, 0)));

	for (int e = 0; e < curr_pose.size(); e++)
	{
		anim_pose[e] = rest_pose[e].slerp(anim_t, curr_pose[e]);
	}

	RotationList vQ;
	vector<Vector3d> vT;

	igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);

	igl::dqs(V, W, vQ, vT, U);
	const int dim = C.cols();
	MatrixXd T(BE.rows() * (dim + 1), dim);
	for (int e = 0; e < BE.rows(); e++)
	{
		Affine3d a = Affine3d::Identity();
		a.translate(vT[e]);
		a.rotate(vQ[e]);
		T.block(e * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}

	igl::deform_skeleton(C, BE, T, CT, BET);
	data(snake_obj).set_vertices(U);
	data(snake_obj).set_points(CT, C_colors);
	data(snake_obj).set_edges(CT, BET, E_colors);
	data(snake_obj).compute_normals();
}

void SandBox::setup_snake(std::string snake_path)
{
	using namespace Eigen;
	using namespace std;
	igl::readOBJ(snake_path, V, F);
	U = V;

	C.resize(20, 3);
	BE.resize(19, 2);
	for (int i = 0; i < C.rows(); i++)
	{
		C.row(i) << 0, 0, ((0 + i * 0.08) - 0.76);
		if (i > 0)
			BE.row(i - 1) << i - 1, i;
	}
	direction_change = false;

	igl::directed_edge_parents(BE, P);
	igl::directed_edge_orientations(C, BE, rest_pose);
	curr_pose = rest_pose;
	next_pose = curr_pose;
	for (int i = 0; i < V.rows(); i++)
		V.row(i) = Scaling(1., 1., (double)BE.rows()) * (Vector3d)V.row(i);
	C *= BE.rows();
	calc_w(V, C, W);
	CT = C;
	BET = BE;

	data().set_vertices(V);
	data().compute_normals();
	setup_snake_aabb_trees();

	prev_vQ = rest_pose;
	prev_vT.resize(BE.rows(), Eigen::Vector3d::Zero());
}

void find_closest_edge(const Eigen::Vector3d &v, const std::vector<Eigen::Vector3d> &edges_center, int &edge_id)
{
	edge_id = 0;
	for (int i = 0; i < edges_center.size(); i++)
	{
		if ((edges_center[i] - v).norm() < (edges_center[edge_id] - v).norm())
		{
			edge_id = i;
		}
	}
}

void calc_w(const Eigen::MatrixXd &V, const Eigen::MatrixXd &C, Eigen::MatrixXd &W)
{
	std::vector<Eigen::Vector3d> edges_center;
	edges_center.resize(C.rows() - 1);
	for (int i = 0; i < edges_center.size(); i++)
	{
		edges_center[i] = ((Eigen::Vector3d)(C.row(i) + C.row(i + 1)) / 2);
		// edges_center[i] = C.row(i);
	}
	W.resize(V.rows(), edges_center.size());
	W.fill(0);

	for (int i = 0; i < V.rows(); i++)
	{
		int edge_id = 0;
		find_closest_edge(V.row(i), edges_center, edge_id);

		// Calculate the weights before normalizing
		double w0 = 0, w1 = 0, w2 = 0;
		// if (edge_id != 0)
		// {
		// 	w0 = 1 / (edges_center[edge_id - 1] - (Eigen::Vector3d)V.row(i)).norm();
		// 	w0 = pow(w0, 10);
		// }

		w1 = 1 / (edges_center[edge_id] - (Eigen::Vector3d)V.row(i)).norm();

		// if (edge_id != edges_center.size() - 1)
		// {
		// 	w2 = 1 / (edges_center[edge_id + 1] - (Eigen::Vector3d)V.row(i)).norm();
		// 	w2 = pow(w2, 10);
		// }

		// Update the normalized weights into W
		// if (edge_id != 0)
		// {
		// 	W(i, edge_id - 1) = w0 / (w0 + w1 + w2);
		// }

		W(i, edge_id) = w1 / (w0 + w1 + w2);

		// if (edge_id != edges_center.size() - 1)
		// {
		// 	W(i, edge_id + 1) = w2 / (w0 + w1 + w2);
		// }
	}
}

void SandBox::check_and_handle_snake_intersect()
{
	igl::AABB<Eigen::MatrixXd, 3> *snake_tree;
	Eigen::Matrix4d snake_trans;
	Eigen::Matrix3d snake_rot;
	Eigen::AlignedBox<double, 3> snake_tree_left_box;
	Eigen::AlignedBox<double, 3> snake_tree_right_box;

	igl::AABB<Eigen::MatrixXd, 3> *other_tree;
	Eigen::Matrix4d other_trans;
	Eigen::Matrix3d other_rot;
	Eigen::AlignedBox<double, 3> other_tree_left_box;
	Eigen::AlignedBox<double, 3> other_tree_right_box;
	Eigen::Vector3d collision_color(1, 1, 1);

	for (int i = 8; i < data_list.size(); i++)
	{
		int j = i - 7;
		snake_tree = trees[snake_obj];
		snake_trans = data_list[snake_obj].MakeTransScaled();
		snake_rot = data_list[snake_obj].GetRotation();
		other_tree = trees[j];
		other_trans = CalcParentsTrans(i) * data_list[i].MakeTransScaled();
		other_rot = data_list[i].GetRotation();
		if (recursive_intersects(snake_tree, snake_trans, snake_rot, data_list[snake_obj], other_tree, other_trans, other_rot, data_list[i]))
		{
			if (i == apple_obj)
			{
				mciSendString("close bite", NULL, 0, NULL);
				mciSendString("open \"OST\\bite_apple.mp3\" type mpegvideo alias bite", NULL, 0, NULL);
				mciSendString("play bite", NULL, 0, NULL);
				score += single_score;

				// define random generators
				std::random_device rd;
				std::mt19937 gen(rd());
				std::uniform_real_distribution<> distr_real(-20, 20);

				// setup the apple location
				Eigen::Vector3d trans = data_list[apple_obj].MakeTransd().col(3).head(3);
				Eigen::Vector3d location(distr_real(gen), distr_real(gen), distr_real(gen));
				data(apple_obj).MyTranslate(location - trans, true);
			}
			else
			{
				mciSendString("close hiss", NULL, 0, NULL);
				mciSendString("open \"OST\\snake_hiss.mp3\" type mpegvideo alias hiss", NULL, 0, NULL);
				mciSendString("play hiss", NULL, 0, NULL);
				game_over = true;
			}
			break;
		}
	}
}

bool SandBox::recursive_intersects(igl::AABB<Eigen::MatrixXd, 3> *tree1, Eigen::Matrix4d trans1, Eigen::Matrix3d rot1, igl::opengl::ViewerData &data1, igl::AABB<Eigen::MatrixXd, 3> *tree2, Eigen::Matrix4d trans2, Eigen::Matrix3d rot2, igl::opengl::ViewerData &data2)
{
	if (boxes_intersect(tree1->m_box, tree2->m_box, trans1, trans2, rot1, rot2))
	{
		if (tree1->is_leaf() && tree2->is_leaf())
		{
			return true;
		}

		igl::AABB<Eigen::MatrixXd, 3> *tree1_left = tree1;
		igl::AABB<Eigen::MatrixXd, 3> *tree1_right = tree1;

		igl::AABB<Eigen::MatrixXd, 3> *tree2_left = tree2;
		igl::AABB<Eigen::MatrixXd, 3> *tree2_right = tree2;

		if (!tree1->is_leaf())
		{
			tree1_left = tree1->m_left;
			tree1_right = tree1->m_right;
		}
		if (!tree2->is_leaf())
		{
			tree2_left = tree2->m_left;
			tree2_right = tree2->m_right;
		}

		return recursive_intersects(tree1_left, trans1, rot1, data1, tree2_left, trans2, rot2, data2) ||
			   recursive_intersects(tree1_left, trans1, rot1, data1, tree2_right, trans2, rot2, data2) ||
			   recursive_intersects(tree1_right, trans1, rot1, data1, tree2_left, trans2, rot2, data2) ||
			   recursive_intersects(tree1_right, trans1, rot1, data1, tree2_right, trans2, rot2, data2);
	}
}

void SandBox::add_box(igl::opengl::ViewerData &data, const Eigen::AlignedBox<double, 3> &box, const Eigen::RowVector3d &color)
{
	/*
	Box vertices arrange like this
			v5-------v6
		   /|       /|
		  /	|      / |
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
	for (int i = 0; i < 12; i++)
		C.row(i) = color;

	// Floor edges
	P1.row(0) = v0;
	P2.row(0) = v1;
	P1.row(1) = v1;
	P2.row(1) = v2;
	P1.row(2) = v2;
	P2.row(2) = v3;
	P1.row(3) = v3;
	P2.row(3) = v0;

	// Ceil edges
	P1.row(4) = v4;
	P2.row(4) = v5;
	P1.row(5) = v5;
	P2.row(5) = v6;
	P1.row(6) = v6;
	P2.row(6) = v7;
	P1.row(7) = v7;
	P2.row(7) = v4;

	// Floor-Ceil linking edges
	P1.row(8) = v0;
	P2.row(8) = v4;
	P1.row(9) = v1;
	P2.row(9) = v5;
	P1.row(10) = v2;
	P2.row(10) = v6;
	P1.row(11) = v3;
	P2.row(11) = v7;

	data.add_edges(P1, P2, C);
}

bool SandBox::boxes_intersect(Eigen::AlignedBox<double, 3> A, Eigen::AlignedBox<double, 3> B, Eigen::Matrix4d Atrans, Eigen::Matrix4d Btrans, Eigen::Matrix3d Arot, Eigen::Matrix3d Brot)
{
	Eigen::Vector3d Pa = transform_vec3(Atrans, A.center());
	Eigen::Vector3d Ax = Arot * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d Ay = Arot * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d Az = Arot * Eigen::Vector3d(0, 0, 1);
	double Wa = (A.sizes()[0] / 2) * Atrans(0, 0);
	double Ha = (A.sizes()[1] / 2) * Atrans(1, 1);
	double Da = (A.sizes()[2] / 2) * Atrans(2, 2);

	Eigen::Vector3d Pb = transform_vec3(Btrans, B.center());
	Eigen::Vector3d Bx = Brot * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d By = Brot * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d Bz = Brot * Eigen::Vector3d(0, 0, 1);
	double Wb = (B.sizes()[0] / 2) * Btrans(0, 0);
	double Hb = (B.sizes()[1] / 2) * Btrans(1, 1);
	double Db = (B.sizes()[2] / 2) * Btrans(2, 2);

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

void SandBox::setup_snake_aabb_trees()
{
	using namespace Eigen;
	MatrixXd V = data(snake_obj).V;
	MatrixXi F = data(snake_obj).F;
	MatrixXd head_V;
	MatrixXi head_F;
	int Vs_index = 0;
	std::map<int, int> V_mapping;
	for (int i = 0; i < V.rows(); i++)
	{
		if (V(i, 2) < C(0, 2) || (C(0, 2) <= V(i, 2)) && (V(i, 2) <= C(1, 2)))
		{
			head_V.conservativeResize(head_V.rows() + 1, 3);
			head_V.row(head_V.rows() - 1) = V.row(i);
			V_mapping[i] = Vs_index++;
		}
	}

	bool in_first1, in_first2, in_first3, link;
	int v1, v2, v3;
	for (int i = 0; i < F.rows(); i++)
	{
		in_first1 = V_mapping.find(F(i, 0)) != V_mapping.end();
		in_first2 = V_mapping.find(F(i, 1)) != V_mapping.end();
		in_first3 = V_mapping.find(F(i, 2)) != V_mapping.end();
		if (in_first1 && in_first2 && in_first3)
		{
			head_F.conservativeResize(head_F.rows() + 1, 3);
			v1 = V_mapping[F(i, 0)];
			v2 = V_mapping[F(i, 1)];
			v3 = V_mapping[F(i, 2)];
			head_F.row(head_F.rows() - 1) << v1, v2, v3;
		}
	}

	trees.emplace_back(new igl::AABB<Eigen::MatrixXd, 3>());
	trees.back()->init(head_V, head_F);
}

void SandBox::reset(bool reset_score, igl::opengl::ViewerCore &camera)
{
	igl::directed_edge_orientations(C, BE, rest_pose);
	curr_pose = rest_pose;
	next_pose = curr_pose;
	BendSnake();
	Eigen::Vector3d translation = data(snake_obj).MakeTransd().col(3).head(3);
	data(snake_obj).MyTranslate(-translation, true);
	data(region_box).lines = region_lines;
	data(region_box).dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_LINES;
	anim_t = 0;
	if (reset_score)
	{
		score = 0;
		level = 1;
	}
	_axes_sys = FRONT;
	_wall_sys = FRONT;
	_snake_dir = SNAKE_UP;
	_arrow_dir = ARROW_NONE;
	time_passed = 1;
	curr_camera_rotation_dir = 0;
	next_camera_rotation_dir = 0;

	camera.camera_eye << 0, -5, 0;
	camera.camera_up << 0, 0, -1;
}

void SandBox::setup_level(int level)
{
	using namespace Eigen;

	// setup cubemap
	data(cubemap).cubemap_texture = igl::opengl::ViewerData::loadCubemap(faces[level]);

	// setup the score for picking up an apple
	single_score = pow(2, level - 1) * 10;

	// define random generators
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> distr_real(-20, 20);
	std::uniform_int_distribution<> distr_int(0, 2);

	// setup the apple location
	Vector3d trans = data_list[apple_obj].MakeTransd().col(3).head(3);
	Vector3d location(distr_real(gen), distr_real(gen), distr_real(gen));
	data(apple_obj).MyTranslate(location - trans, true);
	setup_data_aabb(data(apple_obj));

	// setup mines
	int mine_diff = mines.size() - (level - 1);
	while (mine_diff > 0)
	{
		erase_mesh(mines.back()[0]);
		selected_data_index = 0;
		parents.erase(parents.begin() + mines.back()[0]);
		mines.pop_back();
		mine_diff--;
	}
	while (mine_diff < 0)
	{
		load_mesh_from_file("data/cube.obj");
		parents.push_back(-1);
		std::srand(std::time(0));
		mines.push_back(std::vector<int>(3));
		mines.back()[0] = data().id;
		mines.back()[1] = distr_int(rd);
		mines.back()[2] = ((std::rand() % 2) * 2) - 1;
		data(mines.back()[0]).set_colors(Eigen::RowVector3d(179. / 255., 0, 0));
		data(mines.back()[0]).MyScale(Eigen::Vector3d(1, 1, 1) * 1.5);
		setup_data_aabb(data(mines.back()[0]));
		mine_diff++;
	}

	// randomize mine locations and update movement axes
	data(region_box).lines = region_lines;
	data(region_box).dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_LINES;
	Eigen::Vector3d mine_position;
	Eigen::MatrixXd edge_point1, edge_point2, color;
	edge_point1.resize(1, 3);
	edge_point2.resize(1, 3);
	color.resize(1, 3);
	color.row(0) << 230. / 255., 60. / 255., 48. / 255.;
	for (int i = 0; i < mines.size(); i++)
	{
		mine_position << distr_real(rd), distr_real(rd), distr_real(rd);
		if (mines[i][1] == 0)
		{
			while (std::abs(mine_position(1) <= 3))
				mine_position << distr_real(rd), distr_real(rd), distr_real(rd);

			edge_point1.row(0) << 21., mine_position.y(), mine_position.z();
			edge_point2.row(0) << -21., mine_position.y(), mine_position.z();
		}
		else if (mines[i][1] == 1)
		{
			while (std::abs(mine_position(0)) <= 3)
				mine_position << distr_real(rd), distr_real(rd), distr_real(rd);

			edge_point1.row(0) << mine_position.x(), 21., mine_position.z();
			edge_point2.row(0) << mine_position.x(), -21., mine_position.z();
		}
		else if (mines[i][1] == 2)
		{
			while (std::abs(mine_position(0)) <= 3 && std::abs(mine_position(1)) <= 3)
				mine_position << distr_real(rd), distr_real(rd), distr_real(rd);

			edge_point1.row(0) << mine_position.x(), mine_position.y(), 21.;
			edge_point2.row(0) << mine_position.x(), mine_position.y(), -21.;
		}
		trans = data(mines[i][0]).MakeTransd().col(3).head(3);
		data(mines[i][0]).MyTranslate(mine_position - trans, true);
		edge_point1 /= 42;
		edge_point2 /= 42;
		data(region_box).add_edges(edge_point1, edge_point2, color);
	}
	mines_speed = (std::log(level) * 42.) / 100.;
	selected_data_index = 0;
}

void SandBox::setup_data_aabb(igl::opengl::ViewerData &data)
{
	trees.emplace_back(new igl::AABB<Eigen::MatrixXd, 3>());
	trees.back()->init(data.V, data.F);
}