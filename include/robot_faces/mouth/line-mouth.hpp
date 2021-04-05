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

    void setExpression(const Expression expression) override;

    void draw(sf::RenderWindow &, const float) override;

    // the fill mouth can be implemented by inheriting the line mouth so we make these protected
protected:
    BezierLine neutral_upper_points;
    BezierLine neutral_lower_points;

    BezierLine sad_upper_points;
    BezierLine sad_lower_points;

    BezierLine scared_upper_points;
    BezierLine scared_lower_points;

    BezierLine angry_upper_points;
    BezierLine angry_lower_points;

    BezierLine happy_upper_points;
    BezierLine happy_lower_points;

    BezierLine shocked_upper_points;
    BezierLine shocked_lower_points;

    BezierLine curr_upper_points;
    BezierLine curr_lower_points;

    BezierLine prev_upper_points;
    BezierLine prev_lower_points;

    BezierLine target_upper_points;
    BezierLine target_lower_points;

    std::vector<sf::Vector2f> upper_bezier_line_points_;
    std::vector<sf::Vector2f> lower_bezier_line_points_;
    sf::CircleShape mouth_fillet_;

    bool is_speaking_;

    std::mt19937 random_engine_;
    std::uniform_real_distribution<float> elongation_dist_;
};

LineMouth::LineMouth() : is_speaking_(false),
                         elongation_dist_(0.0, 1.0)
{

    readBezierPointsFromFile(neutral_upper_points, neutral_lower_points, "/res/mouth/neutral.txt");
    readBezierPointsFromFile(sad_upper_points, sad_lower_points, "/res/mouth/sad.txt");
    readBezierPointsFromFile(scared_upper_points, scared_lower_points, "/res/mouth/scared.txt");
    readBezierPointsFromFile(angry_upper_points, angry_lower_points, "/res/mouth/angry.txt");
    readBezierPointsFromFile(happy_upper_points, happy_lower_points, "/res/mouth/happy.txt");
    readBezierPointsFromFile(shocked_upper_points, shocked_lower_points, "/res/mouth/shocked.txt");

    target_upper_points = neutral_upper_points;
    target_lower_points = neutral_lower_points;

    curr_upper_points = target_upper_points;
    curr_lower_points = target_lower_points;

    prev_upper_points = target_upper_points;
    prev_lower_points = target_lower_points;

    mouth_fillet_.setRadius(LINE_MOUTH_THICKNESS / 2.0f);
    mouth_fillet_.setFillColor(colour_);
    mouth_fillet_.setOrigin(LINE_MOUTH_THICKNESS / 2.0f, LINE_MOUTH_THICKNESS / 2.0f);

    std::random_device rd;
    random_engine_.seed(rd());
}

void LineMouth::setSpeaking(const bool speaking)
{
    is_speaking_ = speaking;
    if (speaking)
    {
        target_upper_points = neutral_upper_points;
        target_lower_points = neutral_lower_points;
    }
    else
    {
        target_upper_points = prev_upper_points;
        target_lower_points = prev_lower_points;
    }
}

void LineMouth::setColour(const sf::Color colour)
{
    Entity::setColour(colour);
    mouth_fillet_.setFillColor(colour);
}

void LineMouth::setExpression(const Expression expression)
{
    switch (expression)
    {

    default:
    case Expression::Neutral:
        target_upper_points = neutral_upper_points;
        target_lower_points = neutral_lower_points;

        break;

    case Expression::Sad:
        target_upper_points = sad_upper_points;
        target_lower_points = sad_lower_points;

        break;

    case Expression::Scared:
        target_upper_points = scared_upper_points;
        target_lower_points = scared_lower_points;

        break;

    case Expression::Angry:
        target_upper_points = angry_upper_points;
        target_lower_points = angry_lower_points;

        break;

    case Expression::Happy:
        target_upper_points = happy_upper_points;
        target_lower_points = happy_lower_points;

        break;

    case Expression::Shocked:
        target_upper_points = shocked_upper_points;
        target_lower_points = shocked_lower_points;
        break;
    }

    prev_upper_points = target_upper_points;
    prev_lower_points = target_lower_points;
}

void LineMouth::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{

    // move curr towards to target
    curr_upper_points.start = curr_upper_points.start + frame_delta_time * SPEED * (target_upper_points.start - curr_upper_points.start);
    curr_upper_points.start_control = curr_upper_points.start_control + frame_delta_time * SPEED * (target_upper_points.start_control - curr_upper_points.start_control);
    curr_upper_points.end_control = curr_upper_points.end_control + frame_delta_time * SPEED * (target_upper_points.end_control - curr_upper_points.end_control);
    curr_upper_points.end = curr_upper_points.end + frame_delta_time * SPEED * (target_upper_points.end - curr_upper_points.end);

    curr_lower_points.start = curr_lower_points.start + frame_delta_time * SPEED * (target_lower_points.start - curr_lower_points.start);
    curr_lower_points.start_control = curr_lower_points.start_control + frame_delta_time * SPEED * (target_lower_points.start_control - curr_lower_points.start_control);
    curr_lower_points.end_control = curr_lower_points.end_control + frame_delta_time * SPEED * (target_lower_points.end_control - curr_lower_points.end_control);
    curr_lower_points.end = curr_lower_points.end + frame_delta_time * SPEED * (target_lower_points.end - curr_lower_points.end);

    if (is_speaking_)
    {
        if (curr_upper_points.closeEnough(target_upper_points, 5.0f))
        {
            float elongation_scale_ = elongation_dist_(random_engine_);

            target_upper_points.start = sf::Vector2f(-0.3f * MOUTH_SIZE.x, 0.0f);
            target_upper_points.start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, -1.0f * elongation_scale_ * MOUTH_SIZE.y);
            target_upper_points.end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, -1.0f * elongation_scale_ * MOUTH_SIZE.y);
            target_upper_points.end = sf::Vector2f(0.3f * MOUTH_SIZE.x, 0.0f);

            target_lower_points.start = sf::Vector2f(-0.3f * MOUTH_SIZE.x, 0.0f);
            target_lower_points.start_control = sf::Vector2f(-0.3f * MOUTH_SIZE.x, 1.0f * elongation_scale_ * MOUTH_SIZE.y);
            target_lower_points.end_control = sf::Vector2f(0.3f * MOUTH_SIZE.x, 1.0f * elongation_scale_ * MOUTH_SIZE.y);
            target_lower_points.end = sf::Vector2f(0.3f * MOUTH_SIZE.x, 0.0f);
        }
    }

    upper_bezier_line_points_ = curr_upper_points.generateCurve();
    lower_bezier_line_points_ = curr_lower_points.generateCurve();

    renderWindow.draw(generateLineWithThickness(upper_bezier_line_points_, colour_, LINE_MOUTH_THICKNESS / 2.0f), transform_);
    renderWindow.draw(generateLineWithThickness(lower_bezier_line_points_, colour_, LINE_MOUTH_THICKNESS / 2.0f), transform_);

    mouth_fillet_.setPosition(upper_bezier_line_points_.front().x, upper_bezier_line_points_.front().y);
    renderWindow.draw(mouth_fillet_, transform_);
    mouth_fillet_.setPosition(upper_bezier_line_points_.back().x, upper_bezier_line_points_.back().y);
    renderWindow.draw(mouth_fillet_, transform_);

    Entity::draw(renderWindow, frame_delta_time);
}

#endif // LINE_MOUTH_HPP