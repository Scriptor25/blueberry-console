#pragma once

#include <string>

#include <imgui/imgui.h>

namespace widget
{
    // Widget

    struct Widget
    {
        virtual ~Widget() {}
        virtual void Render() = 0;

        Widget()
        {
            static int global_id = 1;
            Id = global_id++;
            Open = true;
        }

        Widget &Visible(bool open)
        {
            Open = open;
            return *this;
        }

        int Id;
        bool Open;
    };

    // Diagram
    // types of diagrams:
    //  cake
    //  lines
    //  bars
    //      vertical
    //      horizontal
    //  area

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

    // Display
    // types of displays:
    //  bar
    //      vertical
    //      horizontal
    //  circle
    //      half
    //      full

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

    // Image

    struct Image : Widget
    {
        void Render() override;

        void *Ptr;
        int Width;
        int Height;
    };

    // Joystick
    // types of joysticks:
    //  one knob one slider
    //  two knobs
    //  two sliders
    //  touchpad
    //      circle
    //      rectangle

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

    // Terminal

    struct Terminal : Widget
    {
        void Render() override;
    };
}