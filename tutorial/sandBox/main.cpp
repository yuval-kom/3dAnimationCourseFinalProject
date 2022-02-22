
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Snake Game");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* game_menu = new igl::opengl::glfw::imgui::ImGuiMenu();

  viewer.Init("C:/AnimationCourseEngine/build/tutorial/sandBox/configuration.txt");
  
  Init(*disp, game_menu);

  renderer.init(&viewer, game_menu);
  
  disp->SetRenderer(&renderer);

  renderer.core(0).camera_eye += Eigen::Vector3f(0, 0.1, -6);

  renderer.TranslateCamera(Eigen::Vector3f(0, -4, 20));
  
  disp->launch_rendering(true);
  delete game_menu;
  delete disp;
}
