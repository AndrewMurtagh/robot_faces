#ifndef PUPIL_HPP
#define PUPIL_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/shapes/squircle-shape.hpp>

class Pupil : public Entity
{
public:
    Pupil() : show_pupil_highlight_(true)
    {

        squircle_shape_.setFillColor(sf::Color(0, 0, 0, 255));
        squircle_shape_.setOrigin(sf::Vector2f(PUPIL_SIZE / 2.0f, PUPIL_SIZE / 2.0f));
        squircle_shape_.setRadius(sf::Vector2f(PUPIL_SIZE / 2.0f * PUPIL_RADIUS.x, PUPIL_SIZE / 2.0f * PUPIL_RADIUS.y));
        squircle_shape_.setSize(sf::Vector2f(PUPIL_SIZE, PUPIL_SIZE));

        pupil_highlight_shape_one_.setFillColor(sf::Color(255, 255, 255, 50));
        pupil_highlight_shape_one_.setOrigin(sf::Vector2f(PUPIL_SIZE / 4.0f, PUPIL_SIZE / 4.0f));
        pupil_highlight_shape_one_.setRadius(PUPIL_SIZE / 4.0f);

        pupil_highlight_shape_two_.setFillColor(sf::Color(180, 180, 180, 255));
        pupil_highlight_shape_two_.setOrigin(sf::Vector2f(PUPIL_SIZE / 8.0f, PUPIL_SIZE / 8.0f));
        pupil_highlight_shape_two_.setRadius(PUPIL_SIZE / 8.0f);
    }

    void setPupilHighlightShow(const bool show)
    {
        show_pupil_highlight_ = show;
    }

    void setSquircleRadius(const sf::Vector2f squircle_radius)
    {
        squircle_shape_.setRadius(sf::Vector2f(PUPIL_SIZE / 2.0f * squircle_radius.x, PUPIL_SIZE / 2.0f * squircle_radius.y));
    }

    void setColour(const sf::Color colour) override
    {
        Entity::setColour(colour);
        squircle_shape_.setFillColor(colour);
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        renderWindow.draw(squircle_shape_, transform_);

        if (show_pupil_highlight_)
        {
            sf::Transform highlight_one_transform = transform_;
            highlight_one_transform.translate(12.0f, -12.0f);
            renderWindow.draw(pupil_highlight_shape_one_, highlight_one_transform);

            sf::Transform highlight_two_transform = transform_;
            highlight_two_transform.translate(0.0f, -24.0f);
            renderWindow.draw(pupil_highlight_shape_two_, highlight_two_transform);
        }

        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    bool show_pupil_highlight_;
    sf::SquircleShape squircle_shape_;
    sf::CircleShape pupil_highlight_shape_one_;
    sf::CircleShape pupil_highlight_shape_two_;
};

#endif // PUPIL_HPP