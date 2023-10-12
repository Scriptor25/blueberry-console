#include "widget.h"

void widget::DisplayBarVertical::Render()
{
    if (ImGui::Begin(("Display##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DisplayBarHorizontal::Render()
{
    if (ImGui::Begin(("Display##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DisplayCircleHalf::Render()
{
    if (ImGui::Begin(("Display##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DisplayCircleFull::Render()
{
    if (ImGui::Begin(("Display##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}
