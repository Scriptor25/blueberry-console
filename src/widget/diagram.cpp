#include "widget.h"

void widget::DiagramCake::Render()
{
    if (ImGui::Begin(("Diagram##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DiagramLines::Render()
{
    ImGui::PushID(Id);
    if (ImGui::Begin(("Diagram##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DiagramBarsVertical::Render()
{
    ImGui::PushID(Id);
    if (ImGui::Begin(("Diagram##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DiagramBarsHorizontal::Render()
{
    ImGui::PushID(Id);
    if (ImGui::Begin(("Diagram##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}

void widget::DiagramArea::Render()
{
    ImGui::PushID(Id);
    if (ImGui::Begin(("Diagram##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}
