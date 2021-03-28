#ifndef SQUIRCLE_MOUTH_HPP
#define SQUIRCLE_MOUTH_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/shapes/squircle-shape.hpp>

class SquircleMouth : public Entity
{
public:
    SquircleMouth()
    {

        squircle_shape_.setFillColor(sf::Color(255,0,0,255));
        squircle_shape_.setOrigin(sf::Vector2f(NOSE_SIZE.x / 2.0f, NOSE_SIZE.y / 2.0f));
        squircle_shape_.setRadius(sf::Vector2f(NOSE_SIZE.x / 2.0f * SQUIRCLE_NOSE_RADIUS.x, NOSE_SIZE.y / 2.0f * SQUIRCLE_NOSE_RADIUS.y));
        squircle_shape_.setSize(sf::Vector2f(NOSE_SIZE.x, NOSE_SIZE.y));

    }

    void setSquircleRadius(const sf::Vector2f squircle_radius)
    {
        squircle_shape_.setRadius(sf::Vector2f(NOSE_SIZE.x / 2.0f * squircle_radius.x, NOSE_SIZE.y / 2.0f * squircle_radius.y));
    }

    void setSpeaking(const bool speaking)
    {
    }
    

    void setColour(const sf::Color colour) override
    {
        Entity::setColour(colour);
        squircle_shape_.setFillColor(colour);
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        renderWindow.draw(squircle_shape_, transform_);
        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    sf::SquircleShape squircle_shape_;
};

#endif // SQUIRCLE_MOUTH_HPP