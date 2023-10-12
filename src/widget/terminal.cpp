#include "terminal.h"

#include <imgui/imgui.h>

void widget::Terminal::Render()
{
    if (!Open)
        return;

    ImGui::PushID(Id);
    if (ImGui::Begin("Terminal", &Open))
        ;
    ImGui::End();
    ImGui::PopID();
}