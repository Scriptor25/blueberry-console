#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>
#include <implot/implot.h>

#include "node.h"

using namespace std::chrono_literals;

std::shared_ptr<MainNode> node;
size_t joystick = 0;
bool imgui_controller = false;

std::pair<bool, bool> enable_button;
std::pair<bool, bool> disable_button;

bool orientation = true;

void on_enable()
{
  auto &setmode = node->GetSetMode();
  edu_robot::srv::SetMode_Request::SharedPtr request = std::make_shared<edu_robot::srv::SetMode_Request>();
  request->mode.mode = 2;

  setmode->async_send_request(request);
}

void on_disable()
{
  auto &setmode = node->GetSetMode();
  edu_robot::srv::SetMode_Request::SharedPtr request = std::make_shared<edu_robot::srv::SetMode_Request>();
  request->mode.mode = 1;

  setmode->async_send_request(request);
}

void toggle_imgui_controller()
{
  ImGuiIO &io = ImGui::GetIO();
  imgui_controller = !imgui_controller;
  if (imgui_controller)
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; // Disable GUI Controller Input
  else
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad; // Disable GUI Controller Input;
}

void glfw_error_callback(int error_code, const char *description)
{
  printf("[GLFW 0x%08X] %s\r\n", error_code, description);
}

void glfw_key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  (void)scancode;
  (void)mods;

  if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
    glfwSetWindowShouldClose(window, GLFW_TRUE);

  if (key == GLFW_KEY_E && action == GLFW_RELEASE)
    on_enable();
  if (key == GLFW_KEY_Q && action == GLFW_RELEASE)
    on_disable();
  if (key == GLFW_KEY_T && action == GLFW_RELEASE)
    toggle_imgui_controller();
  if (key == GLFW_KEY_F && action == GLFW_RELEASE)
    orientation = !orientation;
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

void on_imgui()
{
  auto &robot = node->GetRobot();

  if (ImGui::Begin("Help"))
  {
    ImGui::Text("Controls %sinverted", orientation ? "" : "not ");

    if (ImGui::BeginTable("Controls", 2))
    {
      ImGui::TableSetupColumn("Key");
      ImGui::TableSetupColumn("Description");

      ImGui::TableNextColumn();
      ImGui::Text("[T]");
      ImGui::TableNextColumn();
      ImGui::Text("Toggle ImGui Controller Usage");

      ImGui::TableNextColumn();
      ImGui::Text("[E]");
      ImGui::TableNextColumn();
      ImGui::Text("Enable Motor Control");

      ImGui::TableNextColumn();
      ImGui::Text("[Q]");
      ImGui::TableNextColumn();
      ImGui::Text("Disable Motor Control");

      ImGui::TableNextColumn();
      ImGui::Text("[F]");
      ImGui::TableNextColumn();
      ImGui::Text("Invert Motor Control");

      ImGui::TableNextColumn();
      ImGui::Text("[Esc]");
      ImGui::TableNextColumn();
      ImGui::Text("Exit Program");

      ImGui::EndTable();
    }
  }
  ImGui::End();

  if (ImGui::Begin("Status Report"))
  {
    ImGui::Text("Latest Barcode: %s", node->GetBarcode().c_str());
    if (ImPlot::BeginPlot("Power", ImVec2(-1, -1)))
    {
      ImPlot::PlotLine("Drive Current", robot.DriveCurrent.data(), robot.DriveCurrent.size());
      ImPlot::PlotLine("Drive Voltage", robot.DriveVoltage.data(), robot.DriveVoltage.size());
      ImPlot::PlotLine("MCU Current", robot.MCUCurrent.data(), robot.MCUCurrent.size());
      ImPlot::PlotLine("MCU Voltage", robot.MCUVoltage.data(), robot.MCUVoltage.size());
      ImPlot::PlotLine("Temperature", robot.Temperature.data(), robot.Temperature.size());

      ImPlot::EndPlot();
    }
  }
  ImGui::End();

  if (ImGui::Begin("Joysticks"))
  {
    ImGui::Text("ImGui %susing controller", imgui_controller ? "" : "not ");
    if (ImGui::BeginCombo("Select", glfwJoystickPresent(joystick) ? glfwGetJoystickName(joystick) : "<disconnected>"))
    {
      for (size_t i = 0; i < 16; i++)
      {
        ImGui::PushID(i);
        bool selected = i == joystick;
        const char *name = glfwJoystickPresent(i) ? glfwGetJoystickName(i) : "<disconnected>";
        if (ImGui::Selectable(name))
          joystick = i;
        if (selected)
          ImGui::SetItemDefaultFocus();

        ImGui::PopID();
      }
      ImGui::EndCombo();
    }
  }
  ImGui::End();
}

void on_input()
{
  if (!glfwJoystickPresent(joystick))
    return;

  geometry_msgs::msg::Twist msg;

  int axes_count;
  int buttons_count;
  auto axes = glfwGetJoystickAxes(joystick, &axes_count);
  auto buttons = glfwGetJoystickButtons(joystick, &buttons_count);

  /*for (int i = 0; i < axes_count; i++)
    printf("%f ", axes[i]);
  printf("\r\n");
  for (int i = 0; i < buttons_count; i++)
    printf("%d ", buttons[i]);
  printf("\r\n");*/

  enable_button.first = enable_button.second;
  enable_button.second = buttons[0];

  disable_button.first = disable_button.second;
  disable_button.second = buttons[1];

  if (enable_button.first && !enable_button.second)
    on_enable();
  if (disable_button.first && !disable_button.second)
    on_disable();

  float gas = axes[5] * 0.5 + 0.5;
  msg.linear.x = axes[1] * gas * (orientation ? -1 : 1);
  msg.angular.z = -axes[0] * gas;

  node->GetVelPub()->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

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
  glfwSetKeyCallback(window, glfw_key_callback);

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

  node = std::make_shared<MainNode>("/eduard/status_report", "/eduard/cmd_vel", "/barcode", "/eduard/set_mode");

  std::thread ros_thread([]
                         { rclcpp::spin(node); });

  rclcpp::on_shutdown([&ros_thread]
                      { ros_thread.~thread(); });

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // IF using Docking Branch
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;   // Enable Multi Viewports
  io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad; // Disable GUI Controller Input

  io.ConfigDockingTransparentPayload = true;
  io.ConfigViewportsNoDecoration = false;

  ImPlot::CreateContext();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
  ImGui_ImplOpenGL3_Init();

  while (!glfwWindowShouldClose(window))
  {
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

    on_imgui();

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

    on_input();
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();

  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  glfwSetErrorCallback(NULL);

  rclcpp::shutdown();

  return 0;
}
