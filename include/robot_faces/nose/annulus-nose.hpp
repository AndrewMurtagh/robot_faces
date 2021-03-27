#ifndef ANNULUS_NOSE_HPP
#define ANNULUS_NOSE_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>

class AnnulusNose : public Entity
{
public:
    AnnulusNose()
    {
        outer_shape_.setRadius(NOSE_SIZE.x / 2.0f);
        outer_shape_.setOrigin(sf::Vector2f(NOSE_SIZE.x / 2.0f, NOSE_SIZE.x / 2.0f));

        const float ratio = 1.0f - ANNULUS_NOSE_THICKNESS;
        inner_shape_.setRadius(NOSE_SIZE.x / 2.0f *  ratio);
        inner_shape_.setOrigin(sf::Vector2f(NOSE_SIZE.x / 2.0f * ratio, NOSE_SIZE.x / 2.0f * ratio));
    }

    void setBackgroundColour(const sf::Color background_colour)
    {
        inner_shape_.setFillColor(background_colour);
    }

    void setColour(const sf::Color colour) override
    {
        outer_shape_.setFillColor(colour);
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        renderWindow.draw(outer_shape_, transform_);
        renderWindow.draw(inner_shape_, transform_);
        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    sf::CircleShape outer_shape_;
    sf::CircleShape inner_shape_;
};

#endif // ANNULUS_NOSE_HPP