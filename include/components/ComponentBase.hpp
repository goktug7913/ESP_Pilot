#pragma once
#include <string>

struct ComponentBaseData
{
    int id;
    std::string name;
};


class ComponentBase
{
private:
    ComponentBaseData data;
public:
    ComponentBase(ComponentBaseData data);
    ~ComponentBase();
    ComponentBaseData getData();
    virtual void init() = 0;
    virtual void update() = 0;
};