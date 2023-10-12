#include <rclcpp/rclcpp.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "widget/manager.h"
#include "widget/widget.h"

#include "minimal_publisher.h"

void glfw_error_callback(int error_code, const char *description)
{
  printf("[GLFW 0x%08X] %s\r\n", error_code, description);
}

void glfw_window_size_callback(GLFWwindow *window, int width, int height)
{
  (void)window;
  glViewport(0, 0, width, height);
}

void gl_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *userParam)
{
  (void)length;
  (void)userParam;

  const char *s_severity;
  const char *s_type;
  const char *s_source;

  switch (severity)
  {
  case GL_DEBUG_SEVERITY_NOTIFICATION:
    s_severity = "NOTIFICATION";
    break;

  case GL_DEBUG_SEVERITY_LOW:
    s_severity = "LOW";
    break;

  case GL_DEBUG_SEVERITY_MEDIUM:
    s_severity = "MEDIUM";
    break;

  case GL_DEBUG_SEVERITY_HIGH:
    s_severity = "HIGH";
    break;

  default:
    s_severity = "UNDEFINED";
    break;
  }

  switch (type)
  {
  case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
    s_type = "DEPRECATED BEHAVIOR";
    break;

  case GL_DEBUG_TYPE_ERROR:
    s_type = "ERROR";
    break;

  case GL_DEBUG_TYPE_MARKER:
    s_type = "MARKER";
    break;

  case GL_DEBUG_TYPE_OTHER:
    s_type = "OTHER";
    break;

  case GL_DEBUG_TYPE_PERFORMANCE:
    s_type = "PERFORMANCE";
    break;

  case GL_DEBUG_TYPE_POP_GROUP:
    s_type = "POP GROUP";
    break;

  case GL_DEBUG_TYPE_PORTABILITY:
    s_type = "PORTABILITY";
    break;

  case GL_DEBUG_TYPE_PUSH_GROUP:
    s_type = "PUSH GROUP";
    break;

  case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
    s_type = "UNDEFINED BEHAVIOR";
    break;

  default:
    s_type = "UNDEFINED";
    break;
  }

  switch (source)
  {
  case GL_DEBUG_SOURCE_API:
    s_source = "API";
    break;

  case GL_DEBUG_SOURCE_APPLICATION:
    s_source = "APPLICATION";
    break;

  case GL_DEBUG_SOURCE_OTHER:
    s_source = "OTHER";
    break;

  case GL_DEBUG_SOURCE_SHADER_COMPILER:
    s_source = "SHADER COMPILER";
    break;

  case GL_DEBUG_SOURCE_THIRD_PARTY:
    s_source = "THIRD PARTY";
    break;

  case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
    s_source = "WINDOW SYSTEM";
    break;

  default:
    s_source = "UNDEFINED";
    break;
  }

  printf("[GL 0x%08X] %s %s from %s: %s\r\n", id, s_severity, s_type, s_source, message);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::thread ros_thread(
      []()
      {
        auto publisher = std::make_shared<MinimalPublisher>();
        rclcpp::spin(publisher);
      });

  rclcpp::on_shutdown([&ros_thread]()
                      { ros_thread.~thread(); });

  // create GLFW window
  glfwSetErrorCallback(glfw_error_callback);
  int error = glfwInit();
  if (error != GLFW_TRUE)
  {
    printf("failed to init GLFW");
    return 1;
  }

  glfwDefaultWindowHints();
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);

  GLFWwindow *window = glfwCreateWindow(800, 600, "Blueberry Node", NULL, NULL);
  if (!window)
  {
    printf("failed to create GLFW window");
    return 1;
  }

  glfwSetWindowSizeCallback(window, glfw_window_size_callback);

  glfwMakeContextCurrent(window);

  error = glewInit();
  if (error != GLEW_OK)
  {
    printf("failed to init GLEW");
    return 1;
  }

  glDebugMessageCallback(gl_debug_callback, NULL);
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

  glClearColor(0.1f, 0.4f, 0.8f, 1.0f);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // IF using Docking Branch
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi Viewports

  io.ConfigDockingTransparentPayload = true;
  io.ConfigViewportsNoDecoration = false;

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
  ImGui_ImplOpenGL3_Init();

  widget::Manager manager;
  manager.RegisterWidget(std::make_shared<widget::Terminal>());
  manager.RegisterWidget(std::make_shared<widget::Image>());
  manager.RegisterWidget(std::make_shared<widget::Image>());
  manager.RegisterWidget(std::make_shared<widget::Image>());

  while (!glfwWindowShouldClose(window))
  {
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::ShowDemoWindow(); // Show demo window! :)

    manager.Render();

    glClear(GL_COLOR_BUFFER_BIT);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
      // Update and Render additional Platform Windows
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(window);
    }

    glfwPollEvents();
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  glfwSetErrorCallback(NULL);

  rclcpp::shutdown();

  return 0;
}