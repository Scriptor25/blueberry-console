#include "image.h"

#include <imgui/imgui.h>

void widget::Image::Render()
{
    if (!Open)
        return;

    ImGui::PushID(Id);
    if (ImGui::Begin("Image", &Open))
        ImGui::Image(Ptr, ImVec2(Width, Height));
    ImGui::End();
    ImGui::PopID();
}