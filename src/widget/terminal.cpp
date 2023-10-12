#include "widget.h"

void widget::Terminal::Render()
{
    if (ImGui::Begin(("Terminal##widget_" + std::to_string(Id)).c_str(), &Open))
        ;
    ImGui::End();
}