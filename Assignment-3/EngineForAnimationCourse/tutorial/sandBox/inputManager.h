#pragma once
#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/glfw/Renderer.h"
#include "sandBox.h"
//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
//#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
//#include <../imgui/imgui.h>

void sendBallToTip(SandBox *scn, int tip_idx);

static void glfw_mouse_press(GLFWwindow *window, int button, int action, int modifier)
{

	Renderer *rndr = (Renderer *)glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer *scn = rndr->GetScene();

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);

		double depth, closestZ = 1;
		int i = 0, savedIndx = scn->selected_data_index, lastIndx = scn->selected_data_index;

		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			depth = rndr->Picking(x2, y2);
			if (depth < 0 && (closestZ > 0 || closestZ < depth))
			{
				savedIndx = i;
				closestZ = depth;
				std::cout << "found " << depth << std::endl;
			}
		}
		scn->selected_data_index = savedIndx;
		scn->data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
		if (lastIndx != savedIndx)
			scn->data_list[lastIndx].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));

		rndr->UpdatePosition(x2, y2);
	}
	else
	{
		rndr->GetScene()->isPicked = false;
	}
}

// static void glfw_char_mods_callback(GLFWwindow* window, unsigned int codepoint, int modifier)
//{
//   __viewer->key_pressed(codepoint, modifier);
// }

void glfw_mouse_move(GLFWwindow *window, double x, double y)
{
	Renderer *rndr = (Renderer *)glfwGetWindowUserPointer(window);
	rndr->UpdatePosition(x, y);
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
	}
	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
	}
}

static void glfw_mouse_scroll(GLFWwindow *window, double x, double y)
{
	Renderer *rndr = (Renderer *)glfwGetWindowUserPointer(window);
	if (rndr->IsPicked())
	{
		igl::opengl::ViewerData *curr_data = &rndr->GetScene()->data();
		if (rndr->GetScene()->selected_data_index > rndr->GetScene()->dest_idx)
			curr_data = &rndr->GetScene()->data_list[rndr->GetScene()->first_link_idx];

		curr_data->TranslateInSystem(rndr->GetScene()->GetRotation(), Eigen::Vector3d(0, 0, 0.1 * y));
	}
	else
		rndr->GetScene()->TranslateInSystem(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.1 * y));
}

void glfw_window_size(GLFWwindow *window, int width, int height)
{
	Renderer *rndr = (Renderer *)glfwGetWindowUserPointer(window);
	// igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	rndr->post_resize(window, width, height);
}

// static void glfw_drop_callback(GLFWwindow *window,int count,const char **filenames)
//{
//
// }

// static void glfw_error_callback(int error, const char* description)
//{
//	fputs(description, stderr);
// }

static void glfw_key_callback(GLFWwindow *window, int key, int scancode, int action, int modifier)
{
	Renderer *rndr = (Renderer *)glfwGetWindowUserPointer(window);
	SandBox *scn = (SandBox *)rndr->GetScene();
	bool limit = scn->selected_data_index == scn->first_link_idx ? false : true;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if (action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case '1':
		{
			sendBallToTip(scn, 1);
			break;
		}
		case '2':
		{
			sendBallToTip(scn, 2);
			break;
		}
		case '3':
		{
			sendBallToTip(scn, 3);
			break;
		}
		case '4':
		{
			sendBallToTip(scn, 4);
			break;
		}
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case '[':
		case ']':
		{
			rndr->ChangeCamera(key);
			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		case 'w':
		case 'W':
			rndr->TranslateCamera(Eigen::Vector3f(0, 0, 0.03f));
			break;
		case 's':
		case 'S':
			rndr->TranslateCamera(Eigen::Vector3f(0, 0, -0.03f));
			break;
		case 't':
		case 'T':
			scn->print_tip_positions();
			break;
		case 'd':
		case 'D':
			scn->print_destination();
			break;
		case 'p':
		case 'P':
			scn->print_transformations();
			break;
		case 'r':
		case 'R':
			scn->reverse_rotation = !scn->reverse_rotation;
			break;
		case GLFW_KEY_UP:
			if (true)
				scn->data().EulerRotation(Eigen::Vector3d(1, 0, 0), 0.05, limit & scn->isLimited);
			else
				scn->MyRotate(Eigen::Vector3d(1, 0, 0), 0.1);
			break;
		case GLFW_KEY_DOWN:
			if (true)
				scn->data().EulerRotation(Eigen::Vector3d(1, 0, 0), -0.05, limit & scn->isLimited);
			else
				scn->MyRotate(Eigen::Vector3d(1, 0, 0), -0.1);
			break;
		case GLFW_KEY_LEFT:
			if (true)
				scn->data().EulerRotation(Eigen::Vector3d(0, 0, 1), -0.05, false);
			else
				scn->MyRotate(Eigen::Vector3d(0, 0, 1), 0.1);
			break;
		case GLFW_KEY_RIGHT:
			if (true)
				
				scn->data().EulerRotation(Eigen::Vector3d(0, 0, 1), 0.05, false);
			else
				scn->MyRotate(Eigen::Vector3d(0, 0, 1), -0.1);
			break;
		case ' ':
			scn->isActive = !scn->isActive;
			break;
		case 'c':
		case 'C':
			scn->FABRIK = !scn->FABRIK;
			if (scn->FABRIK)
				std::cout << "IK solver: FABRIK" << std::endl;
			else
				std::cout << "IK solver: CCD" << std::endl;
			break;
		case 'b':
		case 'B':
			scn->isLimited = !scn->isLimited;
			if (scn->isLimited)
				std::cout << "Rotation limited" << std::endl;
			else
				std::cout << "Rotation unlimited" << std::endl;
			break;
		default:
			Eigen::Vector3f shift;
			float scale;
			rndr->core().get_scale_and_shift_to_fit_mesh(scn->data().V, scn->data().F, scale, shift);

			std::cout << "near " << rndr->core().camera_dnear << std::endl;
			std::cout << "far " << rndr->core().camera_dfar << std::endl;
			std::cout << "angle " << rndr->core().camera_view_angle << std::endl;
			std::cout << "base_zoom " << rndr->core().camera_base_zoom << std::endl;
			std::cout << "zoom " << rndr->core().camera_zoom << std::endl;
			std::cout << "shift " << shift << std::endl;
			std::cout << "translate " << rndr->core().camera_translation << std::endl;

			break; // do nothing
		}
}

void sendBallToTip(SandBox *scn, int tip_idx)
{
	int ball_idx = scn->dest_idx;
	scn->data_list[ball_idx].MyTranslate(-scn->data_list[ball_idx].GetTranslation(), true);
	scn->data_list[ball_idx].MyTranslate((scn->CalcParentsTrans(tip_idx) * scn->data_list[tip_idx].MakeTransd() * Eigen::Vector4d(0, 0, -0.8, 1)).head(3), true);
}

void Init(Display &display, igl::opengl::glfw::imgui::ImGuiMenu *menu)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
	menu->init(&display);
}
