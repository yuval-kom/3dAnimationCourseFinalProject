
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Snake Game");
  Renderer renderer;

  SandBox viewer;
  
  //igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  igl::opengl::glfw::imgui::ImGuiMenu* game_menu = new igl::opengl::glfw::imgui::ImGuiMenu();

  viewer.Init("C:/3dAnimationCourseFinalProject/build/tutorial/sandBox/configuration.txt");
  
  /*viewer.MyRotate(Eigen::Vector3d(2, 0.041, 0.03), 85.9);
  viewer.MyRotate(Eigen::Vector3d(0, 1, 0), 190);*/
  Init(*disp, game_menu);
  game_menu->init_game_menu(disp);

  renderer.init(&viewer, game_menu);
  
  disp->SetRenderer(&renderer);
  /*right_view = renderer.append_core(Eigen::Vector4f((1200 / 4) * 3, 800 / 5, 1200 / 4 * 1, 800 / 5));
  renderer.core(0).camera_center = Eigen::Vector3f(0, 1, 0);
  renderer.core(0).camera_eye = Eigen::Vector3f(0, 0, 0);
  renderer.core(0).camera_up = Eigen::Vector3f(0, 1, 1);*/

  //viewer.MyTranslate(Eigen::Vector3d(0, -5, -10), false); // Camera initalization
  
  disp->launch_rendering(true);
  delete game_menu;
  delete disp;
}
