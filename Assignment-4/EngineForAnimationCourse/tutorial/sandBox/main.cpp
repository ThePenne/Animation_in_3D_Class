#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 1000, "Sanke 3D");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  
  Init(*disp, menu);
  renderer.init(&viewer,1,menu);
  
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}