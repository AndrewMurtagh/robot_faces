#ifndef SQUIRCLE_ENTITY_HPP
#define SQUIRCLE_ENTITY_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/shapes/squircle-shape.hpp>

class SquircleEntity : public Entity
{
public:
    SquircleEntity(sf::Vector2f, sf::Vector2f);

    void setSquircleRadius(const sf::Vector2f);

    void setColour(const sf::Color) override;

    void draw(sf::RenderWindow &, const float) override;

protected:
    sf::SquircleShape squircle_shape_;
    sf::Vector2f size_;
    sf::Vector2f radius_;
};

SquircleEntity::SquircleEntity(sf::Vector2f size, sf::Vector2f radius) : size_(size), radius_(radius)
{

    squircle_shape_.setFillColor(sf::Color(0, 0, 0, 255));
    squircle_shape_.setOrigin(sf::Vector2f(size_.x / 2.0f, size_.y / 2.0f));
    squircle_shape_.setRadius(sf::Vector2f(size_.x / 2.0f * radius_.x, size_.y / 2.0f * radius_.y));
    squircle_shape_.setSize(size_);
}

void SquircleEntity::setSquircleRadius(const sf::Vector2f squircle_radius)
{
    squircle_shape_.setRadius(sf::Vector2f(size_.x / 2.0f * squircle_radius.x, size_.y / 2.0f * squircle_radius.y));
}

void SquircleEntity::setColour(const sf::Color colour)
{
    Entity::setColour(colour);
    squircle_shape_.setFillColor(colour);
}

void SquircleEntity::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
    renderWindow.draw(squircle_shape_, transform_);
    Entity::draw(renderWindow, frame_delta_time);
}

#endif // SQUIRCLE_ENTITY_HPP