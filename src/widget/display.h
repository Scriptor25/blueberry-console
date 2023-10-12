#pragma once

#include "widget.h"

// types of displays:
//  bar
//      vertical
//      horizontal
//  circle
//      half
//      full

namespace widget
{
    struct DisplayBarVertical : Widget
    {
        void Render() override;
    };

    struct DisplayBarHorizontal : Widget
    {
        void Render() override;
    };

    struct DisplayCircleHalf : Widget
    {
        void Render() override;
    };

    struct DisplayCircleFull : Widget
    {
        void Render() override;
    };
}
