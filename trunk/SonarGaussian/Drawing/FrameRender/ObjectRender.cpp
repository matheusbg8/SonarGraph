#include "ObjectRender.h"


ObjectRender::ObjectRender():
    _visible(true)
{
}

ObjectRender::~ObjectRender()
{

}

void ObjectRender::setVisible(bool visible)
{
    _visible = visible;
}

bool ObjectRender::isVisible()
{
    return _visible;
}
