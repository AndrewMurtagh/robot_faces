#ifndef ENTITY_H
#define ENTITY_H

#include <SFML/Graphics.hpp>

#include <ros/ros.h>

#include <robot_faces/entities/ientity.hpp>
#include <robot_faces/utils.hpp>

class Entity : public IEntity
{
public:
    Entity();

    Entity(bool);

    void setTransformation(const sf::Transform) override;

    void setColour(const sf::Color) override;

    void setExpression(const int) override;

    void draw(sf::RenderWindow &, const float);

protected:
    bool mirrored_;
    sf::Transform transform_;
    sf::Color colour_;
};

Entity::Entity() : mirrored_(false)
{
}

Entity::Entity(bool mirror) : mirrored_(mirror)
{
}

void Entity::setTransformation(const sf::Transform transform)
{
    transform_ = transform;
}

void Entity::setColour(const sf::Color colour)
{
    colour_ = colour;
}

void Entity::setExpression(const int)
{
}

void Entity::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
}

#endif // ENTITY_H