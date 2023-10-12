#include "widget.h"

void widget::JoystickOneKnobOneSlider::Render()
{
    if (ImGui::Begin(("Joystick##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::JoystickTwoKnobs::Render()
{
    if (ImGui::Begin(("Joystick##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::JoystickTwoSliders::Render()
{
    if (ImGui::Begin(("Joystick##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::JoystickTouchpadCircle::Render()
{
    if (ImGui::Begin(("Joystick##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::JoystickTouchpadRectangle::Render()
{
    if (ImGui::Begin(("Joystick##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}
