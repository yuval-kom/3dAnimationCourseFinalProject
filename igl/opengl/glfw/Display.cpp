

#include <chrono>
#include <thread>

#include "../gl.h"
#include "Display.h"

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include "igl/opengl/glfw/renderer.h"

#include <external/learnopengl/filesystem.h>
#include <external/learnopengl/shader_m.h>
#include <external/learnopengl/camera.h>
#include <external/learnopengl/model.h>
#include <external/glm/vec3.hpp>
#include <external/glm/matrix.hpp>
#include <external/stb/stb_image.h>

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

float DayLightBoxV[] = {
	-1.0f,  1.0f, -1.0f,
	-1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,

	-1.0f, -1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f, -1.0f,  1.0f,
	-1.0f, -1.0f,  1.0f,

	-1.0f,  1.0f, -1.0f,
	 1.0f,  1.0f, -1.0f,
	 1.0f,  1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f, -1.0f,

	-1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	-1.0f, -1.0f,  1.0f,
	 1.0f, -1.0f,  1.0f
};


static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title)
{
	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);
		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
//#if defined(DEBUG) || defined(_DEBUG)
//		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
//		int major, minor, rev;
//		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
//		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
//		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
//		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
//		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
//		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
//#endif
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();
		
}

bool Display::launch_rendering(bool loop)
{
	//CUBE MAP
	//Shader DaylightBoxShader("C:/FinalProjectAnimation/Shaders/daylightbox.vs", "C:/FinalProjectAnimation/Shaders/daylightbox.fs");
	//unsigned int boxVAO, boxVBO;
	//glGenVertexArrays(1, &boxVAO);
	//glGenBuffers(1, &boxVBO);
	//glBindVertexArray(boxVAO);
	//glBindBuffer(GL_ARRAY_BUFFER, boxVBO);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(DayLightBoxV), &DayLightBoxV, GL_STATIC_DRAW);
	//glEnableVertexAttribArray(0);
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);


	//std::vector<std::string> faces
	//{
	//	//C:\FinalProjectAnimation\textures\DayLightBox
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Right.jpg",
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Left.jpg",
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Top.jpg",
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Bottom.jpg",
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Back.jpg",
	//	"C:/FinalProjectAnimation/textures/DayLightBox/Front.jpg"

	//};

	//unsigned int cubemapTexture = this->loadCubemap(faces);

	//DaylightBoxShader.use();
	//DaylightBoxShader.setInt("daylightbox", 0);


	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	for (int i = 0; i < renderer->GetScene()->data_list.size(); i++)
		renderer->core().toggle(renderer->GetScene()->data_list[i].show_lines);
	while (!glfwWindowShouldClose(window))
	{

		double tic = igl::get_seconds();
		renderer->Animate();
		renderer->draw(window);


		//CUBE MAP
		/*glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)1000 / 800, 0.1f, 100.0f);
		glDepthFunc(GL_LEQUAL);
		DaylightBoxShader.use();
		view = glm::mat4(glm::mat3(camera.GetViewMatrix()));
		DaylightBoxShader.setMat4("view", view);
		DaylightBoxShader.setMat4("projection", projection);
		glBindVertexArray(boxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS);*/

		glfwSwapBuffers(window);
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	
	glfwSetWindowUserPointer(window, userPointer);
	
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

unsigned int Display::loadTexture(char const* path) {
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}

unsigned int Display::loadCubemap(std::vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

