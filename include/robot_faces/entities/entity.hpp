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
        reference_marker_.setRadius(REF_MARKER_RADIUS);
        reference_marker_.setOrigin(REF_MARKER_RADIUS, REF_MARKER_RADIUS);
        reference_marker_.setFillColor(REF_MARKER_COLOUR);
    }

    Entity(bool mirror) : mirrored_(mirror)
    {
        reference_marker_.setRadius(REF_MARKER_RADIUS);
        reference_marker_.setOrigin(REF_MARKER_RADIUS, REF_MARKER_RADIUS);
        reference_marker_.setFillColor(REF_MARKER_COLOUR);
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
        // ROS_INFO("Entity::draw");
        renderWindow.draw(reference_marker_, transform_);
    }

protected:
    bool mirrored_;
    sf::Transform transform_;
    sf::Color colour_;
    sf::CircleShape reference_marker_;
};

#endif // ENTITY_H