#include "widget.h"

void widget::Image::Render()
{
    if (ImGui::Begin(("Image##widget_" + std::to_string(Id)).c_str(), &Open))
        ImGui::Image(Ptr, ImVec2(Width, Height));
    ImGui::End();
}