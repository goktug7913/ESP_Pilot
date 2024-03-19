#include "components/ComponentBase.hpp"

ComponentBase::ComponentBase(ComponentBaseData data)
{
    // We are initializing component data
    this->data = data;
}

ComponentBase::~ComponentBase()
{
    // Destructor
}