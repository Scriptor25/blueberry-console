#pragma once

#include "widget.h"

// types of diagrams:
//  cake
//  lines
//  bars
//      vertical
//      horizontal
//  area

namespace widget
{
    struct DiagramCake : Widget
    {
        void Render() override;
    };

    struct DiagramLines : Widget
    {
        void Render() override;
    };

    struct DiagramBarsVertical : Widget
    {
        void Render() override;
    };

    struct DiagramBarsHorizontal : Widget
    {
        void Render() override;
    };

    struct DiagramArea : Widget
    {
        void Render() override;
    };
}
