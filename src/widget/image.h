#pragma once

#include "widget.h"

namespace widget
{
    struct Image : Widget
    {
        void Render() override;

        void *Ptr;
        int Width;
        int Height;
    };
}
