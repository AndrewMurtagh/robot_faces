#ifndef ANNULUS_NOSE_HPP
#define ANNULUS_NOSE_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>

class AnnulusNose : public Entity
{
public:
    AnnulusNose();

    void setBackgroundColour(const sf::Color);

    void setColour(const sf::Color) override;

    void draw(sf::RenderWindow &, const float) override;

private:
    sf::CircleShape outer_shape_;
    sf::CircleShape inner_shape_;
};

AnnulusNose::AnnulusNose()
{
    outer_shape_.setRadius(NOSE_SIZE.x / 2.0f);
    outer_shape_.setOrigin(sf::Vector2f(NOSE_SIZE.x / 2.0f, NOSE_SIZE.x / 2.0f));

    const float ratio = 1.0f - ANNULUS_NOSE_THICKNESS;
    inner_shape_.setRadius(NOSE_SIZE.x / 2.0f * ratio);
    inner_shape_.setOrigin(sf::Vector2f(NOSE_SIZE.x / 2.0f * ratio, NOSE_SIZE.x / 2.0f * ratio));
}

void AnnulusNose::setBackgroundColour(const sf::Color background_colour)
{
    inner_shape_.setFillColor(background_colour);
}

void AnnulusNose::setColour(const sf::Color colour)
{
    outer_shape_.setFillColor(colour);
}

void AnnulusNose::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
    renderWindow.draw(outer_shape_, transform_);
    renderWindow.draw(inner_shape_, transform_);
    Entity::draw(renderWindow, frame_delta_time);
}

#endif // ANNULUS_NOSE_HPP