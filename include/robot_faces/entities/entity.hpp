#ifndef ENTITY_H
#define ENTITY_H

#include <SFML/Graphics.hpp>

#include <ros/ros.h>

#include <robot_faces/entities/ientity.hpp>
#include <robot_faces/utils.hpp>

class Entity : public IEntity
{
public:
    Entity() : mirrored_(false)
    {
    }

    Entity(bool mirror) : mirrored_(mirror)
    {
    }

    void setTransformation(const sf::Transform transform) override
    {
        transform_ = transform;
    }

    void setColour(const sf::Color colour) override
    {
        colour_ = colour;
    }

    void setExpression(const int) override
    {
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
    }

protected:
    bool mirrored_;
    sf::Transform transform_;
    sf::Color colour_;
};

#endif // ENTITY_H