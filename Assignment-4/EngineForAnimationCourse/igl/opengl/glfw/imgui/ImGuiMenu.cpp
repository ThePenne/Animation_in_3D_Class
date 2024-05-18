// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
////////////////////////////////////////////////////////////////////////////////
//#include "ImGuiMenu.h"
//#include "ImGuiHelpers.h"
#include <igl/project.h>
#include "ImGuiHelpers.h"

#include "ImGuiMenu.h"
#include "../imgui.h"
#include "igl/opengl/glfw/imgui/imgui_impl_glfw.h"
#include "igl/opengl/glfw/imgui/imgui_impl_opengl3.h"

//#include <imgui_fonts_droid_sans.h>
//#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <windows.h>
////////////////////////////////////////////////////////////////////////////////

namespace igl
{
	namespace opengl
	{
		namespace glfw
		{
			namespace imgui
			{
				void read_scores(const std::string &scores_path, std::vector<std::pair<std::string, int>> &score_pairs);
				void write_scores(const std::string &scores_path, const std::vector<std::pair<std::string, int>> &score_pairs);
				IGL_INLINE void ImGuiMenu::init(Display *disp)
				{
					// Setup ImGui binding
					if (disp->window)
					{
						IMGUI_CHECKVERSION();
						if (!context_)
						{
							// Single global context by default, but can be overridden by the user
							static ImGuiContext *__global_context = ImGui::CreateContext();
							context_ = __global_context;
						}
						const char *glsl_version = "#version 150";

						ImGui_ImplGlfw_InitForOpenGL(disp->window, true);
						ImGui_ImplOpenGL3_Init(glsl_version);
						ImGui::GetIO().IniFilename = nullptr;
						ImGui::StyleColorsDark();
						ImGuiStyle &style = ImGui::GetStyle();
						style.FrameRounding = 5.0f;
						reload_font();
					}
				}

				IGL_INLINE void ImGuiMenu::reload_font(int font_size)
				{
					hidpi_scaling_ = hidpi_scaling();
					pixel_ratio_ = pixel_ratio();
					ImGuiIO &io = ImGui::GetIO();
					io.Fonts->Clear();
					// io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data,
					//   droid_sans_compressed_size, font_size * hidpi_scaling_);
					io.FontGlobalScale = 1.0 / pixel_ratio_;
				}

				IGL_INLINE void ImGuiMenu::shutdown()
				{
					// Cleanup
					ImGui_ImplOpenGL3_Shutdown();
					ImGui_ImplGlfw_Shutdown();
					// User is responsible for destroying context if a custom context is given
					// ImGui::DestroyContext(*context_);
				}

				IGL_INLINE bool ImGuiMenu::pre_draw()
				{
					glfwPollEvents();

					// Check whether window dpi has changed
					float scaling = hidpi_scaling();
					if (std::abs(scaling - hidpi_scaling_) > 1e-5)
					{
						reload_font();
						ImGui_ImplOpenGL3_DestroyDeviceObjects();
					}

					ImGui_ImplOpenGL3_NewFrame();
					ImGui_ImplGlfw_NewFrame();
					ImGui::NewFrame();
					return false;
				}

				IGL_INLINE bool ImGuiMenu::post_draw()
				{
					// draw_menu(viewer,core);
					ImGui::Render();
					ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
					return false;
				}

				IGL_INLINE void ImGuiMenu::post_resize(int width, int height)
				{
					if (context_)
					{
						ImGui::GetIO().DisplaySize.x = float(width);
						ImGui::GetIO().DisplaySize.y = float(height);
					}
				}

				// Mouse IO
				IGL_INLINE bool ImGuiMenu::mouse_down(GLFWwindow *window, int button, int modifier)
				{
					ImGui_ImplGlfw_MouseButtonCallback(window, button, GLFW_PRESS, modifier);
					return ImGui::GetIO().WantCaptureMouse;
				}

				IGL_INLINE bool ImGuiMenu::mouse_up(GLFWwindow *window, int button, int modifier)
				{
					// return ImGui::GetIO().WantCaptureMouse;
					//  !! Should not steal mouse up
					return false;
				}

				IGL_INLINE bool ImGuiMenu::mouse_move(GLFWwindow *window, int mouse_x, int mouse_y)
				{
					return ImGui::GetIO().WantCaptureMouse;
				}

				IGL_INLINE bool ImGuiMenu::mouse_scroll(GLFWwindow *window, float delta_y)
				{
					ImGui_ImplGlfw_ScrollCallback(window, 0.f, delta_y);
					return ImGui::GetIO().WantCaptureMouse;
				}

				// Keyboard IO
				IGL_INLINE bool ImGuiMenu::key_pressed(GLFWwindow *window, unsigned int key, int modifiers)
				{
					ImGui_ImplGlfw_CharCallback(nullptr, key);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				IGL_INLINE bool ImGuiMenu::key_down(GLFWwindow *window, int key, int modifiers)
				{
					ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_PRESS, modifiers);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				IGL_INLINE bool ImGuiMenu::key_up(GLFWwindow *window, int key, int modifiers)
				{
					ImGui_ImplGlfw_KeyCallback(window, key, 0, GLFW_RELEASE, modifiers);
					return ImGui::GetIO().WantCaptureKeyboard;
				}

				// Draw menu
				IGL_INLINE void ImGuiMenu::draw_menu(igl::opengl::glfw::Viewer *viewer, std::vector<igl::opengl::ViewerCore> &core)
				{
					// Text labels
					// draw_labels_window(viewer, &core[1]);

					// Viewer settings
					if (callback_draw_viewer_window)
					{
						callback_draw_viewer_window();
					}
					else
					{
						draw_viewer_window(viewer, core);
					}

					// Other windows
					if (callback_draw_custom_window)
					{
						callback_draw_custom_window();
					}
					else
					{
						draw_custom_window();
					}
				}

				IGL_INLINE void ImGuiMenu::draw_viewer_window(igl::opengl::glfw::Viewer *viewer, std::vector<igl::opengl::ViewerCore> &core)
				{
					float menu_width = 180.f * menu_scaling();
					ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
					ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiSetCond_FirstUseEver);
					ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
					bool _viewer_menu_visible = true;
				}

				IGL_INLINE void ImGuiMenu::draw_viewer_menu(igl::opengl::glfw::Viewer *viewer, std::vector<igl::opengl::ViewerCore> &core)
				{
					bool *p_open = NULL;
					static bool no_titlebar = true;
					static bool no_scrollbar = false;
					static bool no_menu = true;
					static bool no_move = true;
					static bool no_resize = true;
					static bool no_collapse = true;
					static bool no_close = true;
					static bool no_nav = false;
					static bool no_background = false;
					static bool no_bring_to_front = false;

					ImGuiWindowFlags window_flags = 0;
					if (no_titlebar)
						window_flags |= ImGuiWindowFlags_NoTitleBar;
					if (no_scrollbar)
						window_flags |= ImGuiWindowFlags_NoScrollbar;
					if (!no_menu)
						window_flags |= ImGuiWindowFlags_MenuBar;
					if (no_move)
						window_flags |= ImGuiWindowFlags_NoMove;
					if (no_resize)
						window_flags |= ImGuiWindowFlags_NoResize;
					if (no_collapse)
						window_flags |= ImGuiWindowFlags_NoCollapse;
					if (no_nav)
						window_flags |= ImGuiWindowFlags_NoNav;
					if (no_background)
						window_flags |= ImGuiWindowFlags_NoBackground;
					if (no_bring_to_front)
						window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;

					auto window_width = core[0].viewport[2];
					auto window_height = core[0].viewport[3];

					ImGui::Begin(
						"Viewer", p_open,
						window_flags);
					ImGui::SetWindowPos(ImVec2(core[0].viewport[0], core[0].viewport[1]), ImGuiCond_Always);
					ImGui::SetWindowSize(ImVec2(window_width, window_height), ImGuiCond_Always);

					std::string player_name;
					int rank;
					std::ostringstream os;
					ImVec2 size;
					switch (menu_mode)
					{
					case main_menu:
						ImGui::SetWindowFontScale(1);
						no_background = false;
						ImGui::SetCursorPosX(window_width * 0.25f);
						ImGui::SetCursorPosY(window_height * 0.25f);
						if (ImGui::Button("Start Game", ImVec2(window_width * 0.5f, 100)))
						{
							menu_mode = game_hud;
							viewer->start_music = true;
							viewer->setup_level(viewer->level);
							viewer->show_all_faces(core[0]);
						}

						ImGui::SetCursorPosX(window_width * 0.25f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 107);
						ImGui::Text("Level:");
						ImGui::SetCursorPosX(window_width * 0.25f + ImGui::CalcTextSize("Level:").x + 10);
						ImGui::SetCursorPosY(window_height * 0.25f + 105);
						ImGui::PushItemWidth(window_width * 0.5f - ImGui::CalcTextSize("Level:").x - 10);
						ImGui::DragInt("", &(viewer->level), 0.03, 1, 10, "%02d");

						ImGui::SetCursorPosX(window_width * 0.25f);
						ImGui::SetCursorPosY(window_height * 0.25f + 130);
						if (ImGui::Button("Scoreboard", ImVec2(window_width * 0.5f, 100)))
						{
							score_pairs.clear();
							read_scores("scores.txt", score_pairs);
							menu_mode = scoreboard;
						}
						ImGui::SetCursorPosX(window_width * 0.25f);
						ImGui::SetCursorPosY(window_height * 0.25f + 235);
						if (ImGui::Button("Exit Game", ImVec2(window_width * 0.5f, 100)))
						{
							viewer->should_close = true;
						}
						break;
					case scoreboard:
						if (ImGui::Button("Back", ImVec2(300, 50)))
						{
							menu_mode = main_menu;
						}
						ImGui::SetCursorPos(ImVec2((window_width - 307), 7));
						if (ImGui::Button("Clear Scores", ImVec2(300, 50)))
						{
							menu_mode = clear_score_prompt;
						}
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize(" SCORES").x) * 0.5f);
						ImGui::SetCursorPosY(window_height * 0.125f);
						ImGui::Text(" SCORES\n########\n");
						rank = 1;
						for (auto score_pair : score_pairs)
						{
							ImGui::SetCursorPosX(window_width / 3.f);
							ImGui::Text("%d. %s - %d", rank++, score_pair.first.c_str(), score_pair.second);
						}
						break;
					case clear_score_prompt:
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize("Are you sure you want to delete all scores?").x) * 0.5f);
						ImGui::SetCursorPosY(window_height * 0.25);
						ImGui::Text("Are you sure you want to delete all the scores?");

						ImGui::SetCursorPosX(window_width * 0.25f - 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Yes, I'm sure", ImVec2(window_width * 0.25f, 100)))
						{
							score_pairs.clear();
							write_scores("scores.txt", score_pairs);
							menu_mode = scoreboard;
						}

						ImGui::SetCursorPosX(window_width * 0.5f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("No, go back", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = scoreboard;
						}
						break;
					case game_hud:
						if (!viewer->isActive){
							ImGui::SetCursorPosX(window_width * 0.5 - ImGui::CalcTextSize("PRESS 'SPACE' TO START").x * 0.5 - 2);
							ImGui::SetCursorPosY(window_height * 0.5 - 5);
							size = ImVec2(window_width * 0.5 + ImGui::CalcTextSize("PRESS 'SPACE' TO START").x * 0.5 + 2, window_height * 0.5 + 25);
							ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetCursorPos(), size, IM_COL32(33, 33, 33, 150));

							ImGui::SetCursorPosX(window_width * 0.5 - ImGui::CalcTextSize("PRESS 'SPACE' TO START").x * 0.5);
							ImGui::SetCursorPosY(window_height * 0.5);
							ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "PRESS 'SPACE' TO START", viewer->level);
						}

						no_background = true;
						ImGui::SetCursorPosX(2);
						ImGui::SetCursorPosY(5);
						size = ImVec2(ImGui::CalcTextSize("SCORE: xxxx").x + 6, 25);
						ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetCursorPos(), size, IM_COL32(33, 33, 33, 150));

						ImGui::SetCursorPosX(5);
						ImGui::SetCursorPosY(5);
						ImGui::SetWindowFontScale(1.5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "SCORE: %04d", viewer->score);

						ImGui::SetCursorPosX(window_width - ImGui::CalcTextSize("LEVEL: xx").x - 8);
						ImGui::SetCursorPosY(5);
						size = ImVec2(window_width - 2, 25);
						ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetCursorPos(), size, IM_COL32(33, 33, 33, 150));

						ImGui::SetCursorPosX(window_width - ImGui::CalcTextSize("LEVEL: xx").x - 5);
						ImGui::SetCursorPosY(5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "LEVEL: %02d", viewer->level);

						ImGui::SetCursorPosX(window_width * 0.25);
						ImGui::SetCursorPosY(window_height - 30);
						ImGui::ProgressBar((double)viewer->time_passed / (double)viewer->max_level_time, ImVec2(window_width * 0.5, 20));
						break;
					case game_over:
						viewer->isActive = false;
						ImGui::SetWindowFontScale(1);
						no_background = false;
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize("GAME OVER").x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f - 50);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text("GAME OVER");
						os << "Score: " << viewer->score;
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize(os.str().c_str()).x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text(os.str().c_str());

						ImGui::SetCursorPosX(window_width * 0.25f - 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Save Score", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = save_score;
						}

						ImGui::SetCursorPosX(window_width * 0.5f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Main Menu", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = main_menu;
							viewer->stop_music = true;
							viewer->reset(true, core[0]);
							viewer->hide_all_faces(core[0]);
						}
						break;
					case save_score:
						ImGui::SetCursorPosX(window_width * 0.25);
						ImGui::SetCursorPosY(window_height * 0.25f);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text("Insert Player Name:");
						ImGui::SetCursorPosX(window_width * 0.25f);
						ImGui::SetCursorPosY(window_height * 0.25f + 20);
						ImGui::PushItemWidth(window_width * 0.5f);
						if (ImGui::InputText("", player_name, ImGuiInputTextFlags_EnterReturnsTrue))
						{
							score_pairs.clear();
							read_scores("scores.txt", score_pairs);
							score_pairs.emplace_back(player_name, viewer->score);
							std::sort(score_pairs.begin(), score_pairs.end(), [=](std::pair<std::string, int> &a, std::pair<std::string, int> &b)
									  { return a.second > b.second; });
							write_scores("scores.txt", score_pairs);
							menu_mode = main_menu;
							viewer->stop_music = true;
							viewer->reset(true, core[0]);
							viewer->hide_all_faces(core[0]);
						}
						break;
					case pause_screen:
						viewer->isActive = false;
						// display score like in hud
						ImGui::SetCursorPosX(5);
						ImGui::SetCursorPosY(5);
						ImGui::SetWindowFontScale(1.5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "SCORE: %d", viewer->score);

						ImGui::SetCursorPosX(window_width - ImGui::CalcTextSize("LEVEL: xx").x - 5);
						ImGui::SetCursorPosY(5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "LEVEL: %02d", viewer->level);

						ImGui::SetWindowFontScale(1);
						// display menu title
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize("GAME PAUSED").x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f - 50);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text("GAME PAUSED");

						// display resume button
						ImGui::SetCursorPosX(window_width * 0.25f - 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Resume", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = game_hud;
						}

						// display quit button
						ImGui::SetCursorPosX(window_width * 0.5f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Quit", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = game_over;
						}
						break;
					case level_finished:
						ImGui::SetCursorPosX(5);
						ImGui::SetCursorPosY(5);
						ImGui::SetWindowFontScale(1.5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "SCORE: %d", viewer->score);

						ImGui::SetCursorPosX(window_width - ImGui::CalcTextSize("LEVEL: xx").x - 5);
						ImGui::SetCursorPosY(5);
						ImGui::TextColored(ImVec4(250. / 255., 208. / 255., 0., 1.), "LEVEL: %02d", viewer->level);

						ImGui::SetWindowFontScale(1);
						// display menu title
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize("FINISHED LEVEL xx!").x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f - 50);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text("FINISHED LEVEL %02d!", viewer->level);

						// display continue button
						ImGui::SetCursorPosX(window_width * 0.25f - 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (viewer->level < 10 && ImGui::Button("Next level", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = game_hud;
							viewer->level++;
							viewer->inital_score = viewer->score;
							viewer->reset(false, core[0]);
							viewer->setup_level(viewer->level);
							viewer->show_all_faces(core[0]);
						}
						else if (viewer->level == 10)
						{
							mciSendString("close win_game", NULL, 0, NULL);
							mciSendString("open \"OST\\win_game_cheering.mp3\" type mpegvideo alias win_game", NULL, 0, NULL);
							mciSendString("play win_game", NULL, 0, NULL);
							menu_mode = win_game;
						}

						// display replay button
						ImGui::SetCursorPosX(window_width * 0.5f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Replay level", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = game_hud;
							viewer->score = viewer->inital_score;
							viewer->reset(false, core[0]);
							viewer->setup_level(viewer->level);
							viewer->show_all_faces(core[0]);
						}

						// display quit button
						ImGui::SetCursorPosX(window_width * 0.375f);
						ImGui::SetCursorPosY(window_height * 0.25f + 160);
						if (ImGui::Button("Quit", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = game_over;
						}
						break;
					case win_game:
						viewer->isActive = false;
						ImGui::SetWindowFontScale(1);
						no_background = false;
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize("A WINNER IS YOU!").x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f - 50);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text("A WINNER IS YOU!");
						os << "Score: " << viewer->score;
						ImGui::SetCursorPosX((window_width - ImGui::CalcTextSize(os.str().c_str()).x) * 0.5);
						ImGui::SetCursorPosY(window_height * 0.25f);
						ImGui::PushItemWidth(window_width * 0.5f);
						ImGui::Text(os.str().c_str());

						ImGui::SetCursorPosX(window_width * 0.25f - 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Save Score", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = save_score;
						}

						ImGui::SetCursorPosX(window_width * 0.5f + 5);
						ImGui::SetCursorPosY(window_height * 0.25f + 50);
						if (ImGui::Button("Main Menu", ImVec2(window_width * 0.25f, 100)))
						{
							menu_mode = main_menu;
							viewer->stop_music = true;
							viewer->reset(true, core[0]);
							viewer->hide_all_faces(core[0]);
						}
						break;
					}
					ImGui::End();
				}

				void read_scores(const std::string &scores_path, std::vector<std::pair<std::string, int>> &score_pairs)
				{
					std::ifstream scores(scores_path);
					std::string score_line, player_name;
					int player_score;
					while (std::getline(scores, score_line))
					{
						player_name = score_line.substr(0, score_line.find(','));
						player_score = atoi((score_line.substr(score_line.find(',') + 1, score_line.size())).c_str());
						score_pairs.emplace_back(player_name, player_score);
					}
					scores.close();
				}
				void write_scores(const std::string &scores_path, const std::vector<std::pair<std::string, int>> &score_pairs)
				{
					std::ofstream scores(scores_path, std::ios::trunc);
					for (auto score_pair : score_pairs)
					{
						scores << score_pair.first << "," << score_pair.second << std::endl;
					}
					scores.close();
				}

				IGL_INLINE void ImGuiMenu::draw_labels_window(igl::opengl::glfw::Viewer *viewer, const igl::opengl::ViewerCore *core)
				{
					// Text labels
					ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiSetCond_Always);
					ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiSetCond_Always);
					bool visible = true;
					ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
					ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
					ImGui::Begin("ViewerLabels", &visible,
								 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoInputs);
					for (const auto &data : viewer->data_list)
					{
						draw_labels(data, core);
					}
					ImGui::End();
					ImGui::PopStyleColor();
					ImGui::PopStyleVar();
				}

				IGL_INLINE void ImGuiMenu::draw_labels(const igl::opengl::ViewerData &data, const igl::opengl::ViewerCore *core)
				{
					if (data.show_vertid)
					{
						for (int i = 0; i < data.V.rows(); ++i)
						{
							draw_text(
								data.V.row(i),
								data.V_normals.row(i),
								std::to_string(i),
								core, data.label_color);
						}
					}

					if (data.show_faceid)
					{
						for (int i = 0; i < data.F.rows(); ++i)
						{
							Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
							for (int j = 0; j < data.F.cols(); ++j)
							{
								p += data.V.row(data.F(i, j));
							}
							p /= (double)data.F.cols();

							draw_text(
								p,
								data.F_normals.row(i),
								std::to_string(i),
								core, data.label_color);
						}
					}

					if (data.labels_positions.rows() > 0)
					{
						for (int i = 0; i < data.labels_positions.rows(); ++i)
						{
							draw_text(
								data.labels_positions.row(i),
								Eigen::Vector3d(0.0, 0.0, 0.0),
								data.labels_strings[i],
								core, data.label_color);
						}
					}
				}

				IGL_INLINE void ImGuiMenu::draw_text(
					Eigen::Vector3d pos,
					Eigen::Vector3d normal,
					const std::string &text,
					const igl::opengl::ViewerCore *core,
					const Eigen::Vector4f color)
				{
					pos += normal * 0.005f * core[1].object_scale;
					Eigen::Vector3f coord = igl::project(Eigen::Vector3f(pos.cast<float>()),
														 core[1].view, core[1].proj, core[1].viewport);

					// Draw text labels slightly bigger than normal text
					ImDrawList *drawList = ImGui::GetWindowDrawList();
					drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.2,
									  ImVec2(coord[0] / pixel_ratio_, (core[1].viewport[3] - coord[1]) / pixel_ratio_),
									  ImGui::GetColorU32(ImVec4(
										  color(0),
										  color(1),
										  color(2),
										  color(3))),
									  &text[0], &text[0] + text.size());
				}

				IGL_INLINE float ImGuiMenu::pixel_ratio()
				{
					// Computes pixel ratio for hidpi devices
					int buf_size[2];
					int win_size[2];
					GLFWwindow *window = glfwGetCurrentContext();
					glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
					glfwGetWindowSize(window, &win_size[0], &win_size[1]);
					return (float)buf_size[0] / (float)win_size[0];
				}

				IGL_INLINE float ImGuiMenu::hidpi_scaling()
				{
					// Computes scaling factor for hidpi devices
					float xscale, yscale;
					GLFWwindow *window = glfwGetCurrentContext();
					glfwGetWindowContentScale(window, &xscale, &yscale);
					return 0.5 * (xscale + yscale);
				}

			} // end namespace
		}	  // end namespace
	}		  // end namespace
} // end namespace
