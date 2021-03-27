#ifndef SQUIRCLE_NOSE_HPP
#define SQUIRCLE_NOSE_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/shapes/squircle-shape.hpp>

class SquircleNose : public Entity
{
public:
    SquircleNose()
    {

        squircle_shape_.setFillColor(sf::Color(0,0,0,255));
        squircle_shape_.setOrigin(sf::Vector2f(NOSE_SIZE / 2.0f, NOSE_SIZE / 2.0f));
        squircle_shape_.setRadius(sf::Vector2f(NOSE_SIZE / 2.0f * SQUIRCLE_NOSE_RADIUS.x, NOSE_SIZE / 2.0f * SQUIRCLE_NOSE_RADIUS.y));
        squircle_shape_.setSize(sf::Vector2f(NOSE_SIZE, NOSE_SIZE));
        squircle_shape_.setCornerPointCount(NUM_CORNER_POINTS);
    }

    void setSquircleRadius(const sf::Vector2f squircle_radius)
    {
        ROS_INFO("SquircleNose::setSquircleRadius");
        squircle_shape_.setRadius(sf::Vector2f(NOSE_SIZE / 2.0f * squircle_radius.x, NOSE_SIZE / 2.0f * squircle_radius.y));
    }

    void setColour(const sf::Color colour) override
    {
        Entity::setColour(colour);
        squircle_shape_.setFillColor(colour);
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        // ROS_INFO("SquircleNose::draw");
        renderWindow.draw(squircle_shape_, transform_);
        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    sf::SquircleShape squircle_shape_;
};

#endif // SQUIRCLE_NOSE_HPP