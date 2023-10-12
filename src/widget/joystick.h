#pragma once

#include "widget.h"

// types of joysticks:
//  one knob one slider
//  two knobs
//  two sliders
//  touchpad
//      circle
//      rectangle

namespace widget
{
    struct JoystickOneKnobOneSlider : Widget
    {
        void Render() override;
    };

    struct JoystickTwoKnobs : Widget
    {
        void Render() override;
    };

    struct JoystickTwoSliders : Widget
    {
        void Render() override;
    };

    struct JoystickTouchpadCircle : Widget
    {
        void Render() override;
    };

    struct JoystickTouchpadRectangle : Widget
    {
        void Render() override;
    };
}
