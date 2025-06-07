#pragma once

template <typename T>
struct Functor
{
    T *obj = nullptr;
    void (T::*method)() = nullptr;

    Functor() = default;

    Functor(T *o, void (T::*m)()) : obj(o), method(m) {}

    void operator()() const
    {
        if (obj && method)
        {
            (obj->*method)();
        }
    }

    bool valid() const
    {
        return obj && method;
    }
};
