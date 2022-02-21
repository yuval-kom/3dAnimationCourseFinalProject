
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

  viewer.Init("C:/AnimationCourseEngine/build/tutorial/sandBox/configuration.txt");
  
  /*viewer.MyRotate(Eigen::Vector3d(2, 0.041, 0.03), 85.9);
  viewer.MyRotate(Eigen::Vector3d(0, 1, 0), 190);*/
  Init(*disp, game_menu);
  //game_menu->init_game_menu(disp);

  renderer.init(&viewer, game_menu);
  
  disp->SetRenderer(&renderer);
  disp->SetRenderer(&renderer);
  renderer.core(0).camera_up = Eigen::Vector3f(float(viewer.tip_position.x()), float(viewer.tip_position.y()), float(viewer.tip_position.z()));
  renderer.RotateCameraX(90);
  viewer.MyTranslate(Eigen::Vector3d(35, 0, 5), false);
  
  disp->launch_rendering(true);
  delete game_menu;
  delete disp;
}
