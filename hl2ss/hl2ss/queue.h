
#pragma once

#include <queue>

// OK
template<typename T>
T pull(std::queue<T>& q)
{
    T e = q.front();
    q.pop();
    return e;
}
