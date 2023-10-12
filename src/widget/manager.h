#pragma once

#include <memory>
#include <vector>

#include "widget.h"

namespace widget
{
    class Manager
    {
    public:
        Manager &RegisterWidget(std::shared_ptr<Widget> widget)
        {
            m_Widgets.push_back(widget);
            return *this;
        }

        Manager &Render()
        {
            // Render Manager Window
            if (ImGui::Begin("Manager##manager_window"))
            {
            }
            ImGui::End();

            // Render Widgets
            for (auto widget : m_Widgets)
                if (widget->Open)
                    widget->Render();

            return *this;
        }

    private:
        std::vector<std::shared_ptr<Widget>> m_Widgets;
    };
}