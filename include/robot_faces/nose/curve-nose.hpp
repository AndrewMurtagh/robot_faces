#ifndef CURVE_NOSE_HPP
#define CURVE_NOSE_HPP

#include <robot_faces/entities/entity.hpp>
#include <robot_faces/utils.hpp>
#include <ros/ros.h>

class CurveNose : public Entity
{
public:
    CurveNose();

    void setTransformation(const sf::Transform) override;

    void setColour(const sf::Color) override;

    void draw(sf::RenderWindow &, const float) override;

private:
    sf::CircleShape left_nose_curve_fillet_;
    sf::CircleShape right_nose_curve_fillet_;
    sf::VertexArray curve_nose_points_;

    void generateCurveNosePoints();
};

CurveNose::CurveNose() : curve_nose_points_(sf::TrianglesStrip)
{
    generateCurveNosePoints();
}

void CurveNose::setTransformation(const sf::Transform transform)
{
    Entity::setTransformation(transform);
    generateCurveNosePoints();
}

void CurveNose::setColour(const sf::Color colour)
{
    Entity::setColour(colour);
    generateCurveNosePoints();
}

void CurveNose::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
    renderWindow.draw(curve_nose_points_, transform_);
    renderWindow.draw(left_nose_curve_fillet_, transform_);
    renderWindow.draw(right_nose_curve_fillet_, transform_);
    Entity::draw(renderWindow, frame_delta_time);
}

void CurveNose::generateCurveNosePoints()
{
    curve_nose_points_.clear();

    const int start_angle = 35;
    const int end_angle = -215;

    sf::Vector2f prev_vector = sf::Vector2f(CURVE_NOSE_RADIUS * cos(degToRad(start_angle)), CURVE_NOSE_RADIUS * sin(degToRad(start_angle)));
    sf::Vector2f curr_vector = sf::Vector2f(CURVE_NOSE_RADIUS * cos(degToRad(start_angle - 1)), CURVE_NOSE_RADIUS * sin(degToRad(start_angle - 1)));

    for (int ang = start_angle; ang >= end_angle; ang -= 1)
    {

        curr_vector = sf::Vector2f(CURVE_NOSE_RADIUS * cos(degToRad(ang)), CURVE_NOSE_RADIUS * sin(degToRad(ang)));

        sf::Vector2f line = prev_vector - curr_vector;
        sf::Vector2f normal = normalize(sf::Vector2f(-line.y, line.x));

        curve_nose_points_.append(sf::Vertex(prev_vector - CURVE_NOSE_THICKNESS * normal, colour_));
        curve_nose_points_.append(sf::Vertex(prev_vector + CURVE_NOSE_THICKNESS * normal, colour_));

        curve_nose_points_.append(sf::Vertex(curr_vector - CURVE_NOSE_THICKNESS * normal, colour_));
        curve_nose_points_.append(sf::Vertex(curr_vector + CURVE_NOSE_THICKNESS * normal, colour_));

        prev_vector = curr_vector;
    }

    left_nose_curve_fillet_.setRadius(CURVE_NOSE_THICKNESS);
    left_nose_curve_fillet_.setFillColor(colour_);
    left_nose_curve_fillet_.setOrigin(CURVE_NOSE_THICKNESS, CURVE_NOSE_THICKNESS);
    left_nose_curve_fillet_.setPosition(CURVE_NOSE_RADIUS * cos(degToRad(start_angle)), CURVE_NOSE_RADIUS * sin(degToRad(start_angle)));

    right_nose_curve_fillet_.setRadius(CURVE_NOSE_THICKNESS);
    right_nose_curve_fillet_.setFillColor(colour_);
    right_nose_curve_fillet_.setOrigin(CURVE_NOSE_THICKNESS, CURVE_NOSE_THICKNESS);
    right_nose_curve_fillet_.setPosition(CURVE_NOSE_RADIUS * cos(degToRad(end_angle)), CURVE_NOSE_RADIUS * sin(degToRad(end_angle)));
}

#endif // CURVE_NOSE_HPP