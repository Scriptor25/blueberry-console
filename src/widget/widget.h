#pragma once

namespace widget
{
    struct Widget
    {
        virtual ~Widget() = 0;
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
}