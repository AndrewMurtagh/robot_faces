#ifndef LINE_MOUTH_HPP
#define LINE_MOUTH_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/utils.hpp>
#include <robot_faces/shapes/squircle-shape.hpp>
#include <robot_faces/shapes/bezierline.hpp>

class LineMouth : public Entity
{
public:
    LineMouth()
    {
        mouth_bezier_control_points_ = {
            .start = sf::Vector2f(-100, 0),
            .end = sf::Vector2f(100, 0),
            .start_control = sf::Vector2f(-30, 20),
            .end_control = sf::Vector2f(30, 20)
        };
        mouth_fillet_.setRadius(LINE_MOUTH_THICKNESS/2.0f);
        mouth_fillet_.setFillColor(colour_);
        mouth_fillet_.setOrigin(LINE_MOUTH_THICKNESS/2.0f, LINE_MOUTH_THICKNESS/2.0f);
    }

    void setColour(const sf::Color colour) override
    {
        Entity::setColour(colour);
        mouth_fillet_.setFillColor(colour);
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        bezier_line_points_ = mouth_bezier_control_points_.generateCurve();

        renderWindow.draw(generateLineWithThickness(bezier_line_points_, colour_, LINE_MOUTH_THICKNESS/2.0f), transform_);
        mouth_fillet_.setPosition(bezier_line_points_.front().x, bezier_line_points_.front().y);
        renderWindow.draw(mouth_fillet_, transform_);
        mouth_fillet_.setPosition(bezier_line_points_.back().x, bezier_line_points_.back().y);
        renderWindow.draw(mouth_fillet_, transform_);

        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    std::vector<sf::Vector2f> bezier_line_points_;
    sf::CircleShape mouth_fillet_;
    BezierLine mouth_bezier_control_points_;
};

#endif // LINE_MOUTH_HPP