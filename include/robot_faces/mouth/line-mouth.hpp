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
    LineMouth();

    void setSpeaking(const bool);

    void setColour(const sf::Color) override;

    void draw(sf::RenderWindow &, const float) override;

private:
    BezierLine upper_mouth_bezier_control_points_;
    BezierLine lower_mouth_bezier_control_points_;
    std::vector<sf::Vector2f> upper_bezier_line_points_;
    std::vector<sf::Vector2f> lower_bezier_line_points_;
    sf::CircleShape mouth_fillet_;

    bool is_speaking_;

    float curr_elongation_scale_;
    float target_elongation_scale_;

    std::mt19937 random_engine_;
    std::uniform_real_distribution<float> elongation_dist_;
};

LineMouth::LineMouth() : is_speaking_(false),
                         curr_elongation_scale_(0.0),
                         target_elongation_scale_(0.0),
                         elongation_dist_(0.0, 1.0)
{

    upper_mouth_bezier_control_points_ = {
        .start = sf::Vector2f(-0.5f * MOUTH_SIZE.x, 0.0f * MOUTH_SIZE.y),
        .end = sf::Vector2f(0.5f * MOUTH_SIZE.x, 0.0f * MOUTH_SIZE.y),
        .start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, curr_elongation_scale_ * MOUTH_SIZE.y),
        .end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, curr_elongation_scale_ * MOUTH_SIZE.y)};

    lower_mouth_bezier_control_points_ = {
        .start = sf::Vector2f(-0.5f * MOUTH_SIZE.x, 0.0f * MOUTH_SIZE.y),
        .end = sf::Vector2f(0.5f * MOUTH_SIZE.x, 0.0f * MOUTH_SIZE.y),
        .start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, -1.0f * curr_elongation_scale_ * MOUTH_SIZE.y),
        .end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, -1.0f * curr_elongation_scale_ * MOUTH_SIZE.y)};

    mouth_fillet_.setRadius(LINE_MOUTH_THICKNESS / 2.0f);
    mouth_fillet_.setFillColor(colour_);
    mouth_fillet_.setOrigin(LINE_MOUTH_THICKNESS / 2.0f, LINE_MOUTH_THICKNESS / 2.0f);

    std::random_device rd;
    random_engine_.seed(rd());

    target_elongation_scale_ = elongation_dist_(random_engine_);
}

void LineMouth::setSpeaking(const bool speaking)
{
    is_speaking_ = speaking;
    if (speaking)
    {
        target_elongation_scale_ = elongation_dist_(random_engine_);
    }
    else
    {
        target_elongation_scale_ = 0.0f;
    }
}

void LineMouth::setColour(const sf::Color colour)
{
    Entity::setColour(colour);
    mouth_fillet_.setFillColor(colour);
}

void LineMouth::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
    if (is_speaking_)
    {
        if (abs(target_elongation_scale_ - curr_elongation_scale_) < SPEAKING_ELON_CLOSE_ENOUGH)
        {
            curr_elongation_scale_ = target_elongation_scale_;
            target_elongation_scale_ = elongation_dist_(random_engine_);
        }
        else
        {
            float delta = frame_delta_time * SPEAKING_SPEED * (target_elongation_scale_ - curr_elongation_scale_);
            curr_elongation_scale_ += delta;
        }

        upper_mouth_bezier_control_points_.start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, curr_elongation_scale_ * MOUTH_SIZE.y);
        upper_mouth_bezier_control_points_.end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, curr_elongation_scale_ * MOUTH_SIZE.y);
        lower_mouth_bezier_control_points_.start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, -1.0f * curr_elongation_scale_ * MOUTH_SIZE.y);
        lower_mouth_bezier_control_points_.end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, -1.0f * curr_elongation_scale_ * MOUTH_SIZE.y);
    }
    upper_bezier_line_points_ = upper_mouth_bezier_control_points_.generateCurve();
    lower_bezier_line_points_ = lower_mouth_bezier_control_points_.generateCurve();

    renderWindow.draw(generateLineWithThickness(upper_bezier_line_points_, colour_, LINE_MOUTH_THICKNESS / 2.0f), transform_);
    renderWindow.draw(generateLineWithThickness(lower_bezier_line_points_, colour_, LINE_MOUTH_THICKNESS / 2.0f), transform_);

    mouth_fillet_.setPosition(upper_bezier_line_points_.front().x, upper_bezier_line_points_.front().y);
    renderWindow.draw(mouth_fillet_, transform_);
    mouth_fillet_.setPosition(upper_bezier_line_points_.back().x, upper_bezier_line_points_.back().y);
    renderWindow.draw(mouth_fillet_, transform_);

    Entity::draw(renderWindow, frame_delta_time);
}

#endif // LINE_MOUTH_HPP