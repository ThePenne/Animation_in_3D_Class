#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/ViewerCore.h"
#include <windows.h>
#include "igl/aabb.h"

#pragma comment(lib, "Winmm.lib")

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	enum Arrow_direction
	{
		ARROW_UP,
		ARROW_DOWN,
		ARROW_LEFT,
		ARROW_RIGHT,
		ARROW_NONE
	};

	enum Snake_direction
	{
		SNAKE_UP,
		SNAKE_DOWN,
		SNAKE_LEFT,
		SNAKE_RIGHT,
	};

	enum Axes_system
	{
		FRONT,
		BACK,
		TOP,
		BOTTOM
	};

	void UpdateSnakeLeft(Eigen::Vector3d &axis);
	void UpdateSnakeRight(Eigen::Vector3d &axis);
	void UpdateSnakeUp(Eigen::Vector3d &axis);
	void UpdateSnakeDown(Eigen::Vector3d &axis);

	SandBox();
	~SandBox();
	void Init(const std::string &config);
	void BendSnake();
	void MoveSnake();
	void MoveMines();
	void setup_snake(std::string snake_path);
	void CalcSnakeUV();
	void UpdateNextPose();
	void RotateCamera(igl::opengl::ViewerCore &camera);
	void StartMusic()
	{
		mciSendString("open \"OST\\Ievan_Polkka(MC).mp3\" type mpegvideo alias music", NULL, 0, NULL);
		mciSendString("play music repeat", NULL, 0, NULL);
		music_playing = true;
	}
	void StopMusic()
	{
		mciSendString("stop music", NULL, 0, NULL);
		mciSendString("close music", NULL, 0, NULL);
		music_playing = false;
	}
	IGL_INLINE void SetDirection(Arrow_direction dir) { _arrow_dir = dir; };
	double doubleVariable;
	bool music_playing{false};
	void ToggleHeadCamera()
	{
		head_camera = !head_camera;
		toggled = true;
	}
	void update_score(int points) { score += points; }

	void check_and_handle_snake_intersect();
	bool recursive_intersects(igl::AABB<Eigen::MatrixXd, 3> *tree1, Eigen::Matrix4d trans1, Eigen::Matrix3d rot1, igl::opengl::ViewerData &data1, igl::AABB<Eigen::MatrixXd, 3> *tree2, Eigen::Matrix4d trans2, Eigen::Matrix3d rot2, igl::opengl::ViewerData &data2);
	void add_box(igl::opengl::ViewerData &data, const Eigen::AlignedBox<double, 3> &box, const Eigen::RowVector3d &color);
	bool boxes_intersect(Eigen::AlignedBox<double, 3> A, Eigen::AlignedBox<double, 3> B, Eigen::Matrix4d Atrans, Eigen::Matrix4d Btrans, Eigen::Matrix3d Arot, Eigen::Matrix3d Brot);

	void setup_snake_aabb_trees();
	void setup_data_aabb(igl::opengl::ViewerData &data);
	void setup_level(int level);
	void setup_region_box();
	void setup_region_walls();

	void set_walls();
	void reset(bool reset_score, igl::opengl::ViewerCore &camera);

	inline void show_all_faces(const igl::opengl::ViewerCore &core)
	{
		for (auto &obj : data_list)
			core.set(obj.show_faces, true);
		core.unset(data_list[region_box].show_faces);
		core.unset(data_list[region_wall_front].show_faces);
	}

	inline void hide_all_faces(const igl::opengl::ViewerCore &core)
	{
		for (auto &obj : data_list)
			core.set(obj.show_faces, false);
	}

	void rotate_camera(int dir)
	{
		next_camera_rotation_dir = dir;
	}

	unsigned int loadCubemap(std::vector<std::string> faces);

private:
	typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
		RotationList;

	void Animate(igl::opengl::ViewerCore &camera);
	RotationList rest_pose, curr_pose, next_pose;
	Eigen::MatrixXd V, W, C, U, C_org, C_dest, CT;
	Eigen::MatrixXd C_colors, E_colors;
	Eigen::MatrixXi F, BE, BET;
	Eigen::VectorXi P;
	bool direction_change;
	bool recompute{true};
	bool head_camera{false};
	bool toggled{false};
	double org_dnear;
	double anim_t{0.0}, anim_t_dif{0.1};
	int curr_camera_rotation_dir{0};
	int next_camera_rotation_dir{0};
	Arrow_direction _arrow_dir{ARROW_NONE};
	Snake_direction _snake_dir{SNAKE_UP};
	Axes_system _axes_sys{FRONT};
	Axes_system _wall_sys{FRONT};
	bool direction_changed;
	bool swivel_dir{0};
	bool camera_dir{0};
	std::vector<igl::AABB<Eigen::MatrixXd, 3> *> trees;
	std::vector<igl::AABB<Eigen::MatrixXd, 3> *> sub_trees;
	std::vector<Eigen::MatrixXd> Vs{2};
	std::vector<Eigen::MatrixXi> Fs{2};
	std::vector<Eigen::MatrixXd> Ws{2};
	size_t snake_obj;
	size_t apple_obj;
	// vector of pairs: first is a mapping into data_list, second is movement direction
	std::vector<std::vector<int>> mines;
	double mines_speed;
	size_t region_wall_front;
	size_t region_wall_back;
	size_t region_wall_left;
	size_t region_wall_right;
	size_t region_wall_top;
	size_t region_wall_bottom;
	size_t region_box;
	Eigen::MatrixXd region_lines;
	RotationList prev_vQ;
	std::vector<Eigen::Vector3d> prev_vT;
	int single_score{0};

	std::vector<std::vector<std::string>> faces{{},
												{"textures/cubemap1/right.jpg",
												 "textures/cubemap1/left.jpg",
												 "textures/cubemap1/top.jpg",
												 "textures/cubemap1/bottom.jpg",
												 "textures/cubemap1/front.jpg",
												 "textures/cubemap1/back.jpg"},
												{"textures/cubemap2/right.jpg",
												 "textures/cubemap2/left.jpg",
												 "textures/cubemap2/top.jpg",
												 "textures/cubemap2/bottom.jpg",
												 "textures/cubemap2/front.jpg",
												 "textures/cubemap2/back.jpg"},
												{"textures/cubemap3/right.jpg",
												 "textures/cubemap3/left.jpg",
												 "textures/cubemap3/top.jpg",
												 "textures/cubemap3/bottom.jpg",
												 "textures/cubemap3/front.jpg",
												 "textures/cubemap3/back.jpg"},
												{"textures/cubemap4/right.jpg",
												 "textures/cubemap4/left.jpg",
												 "textures/cubemap4/top.jpg",
												 "textures/cubemap4/bottom.jpg",
												 "textures/cubemap4/front.jpg",
												 "textures/cubemap4/back.jpg"},
												{"textures/cubemap5/right.jpg",
												 "textures/cubemap5/left.jpg",
												 "textures/cubemap5/top.jpg",
												 "textures/cubemap5/bottom.jpg",
												 "textures/cubemap5/front.jpg",
												 "textures/cubemap5/back.jpg"},
												{"textures/cubemap6/right.jpg",
												 "textures/cubemap6/left.jpg",
												 "textures/cubemap6/top.jpg",
												 "textures/cubemap6/bottom.jpg",
												 "textures/cubemap6/front.jpg",
												 "textures/cubemap6/back.jpg"},
												{"textures/cubemap7/right.jpg",
												 "textures/cubemap7/left.jpg",
												 "textures/cubemap7/top.jpg",
												 "textures/cubemap7/bottom.jpg",
												 "textures/cubemap7/front.jpg",
												 "textures/cubemap7/back.jpg"},
												{"textures/cubemap8/right.jpg",
												 "textures/cubemap8/left.jpg",
												 "textures/cubemap8/top.jpg",
												 "textures/cubemap8/bottom.jpg",
												 "textures/cubemap8/front.jpg",
												 "textures/cubemap8/back.jpg"},
												{"textures/cubemap9/right.jpg",
												 "textures/cubemap9/left.jpg",
												 "textures/cubemap9/top.jpg",
												 "textures/cubemap9/bottom.jpg",
												 "textures/cubemap9/front.jpg",
												 "textures/cubemap9/back.jpg"},
												{"textures/cubemap10/right.jpg",
												 "textures/cubemap10/left.jpg",
												 "textures/cubemap10/top.jpg",
												 "textures/cubemap10/bottom.jpg",
												 "textures/cubemap10/front.jpg",
												 "textures/cubemap10/back.jpg"}};
};
