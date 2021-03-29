#ifndef IENTITY_H
#define IENTITY_H

#include <SFML/Graphics.hpp>

class IEntity
{
public:
    virtual void setTransformation(const sf::Transform) = 0;
    virtual const sf::Transform getTransformation() const = 0;
    virtual void setColour(const sf::Color) = 0;
    virtual void setExpression(const int) = 0;
    virtual void draw(sf::RenderWindow &, const float) = 0;
};

#endif // IENTITY_H