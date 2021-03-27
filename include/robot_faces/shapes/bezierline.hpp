#ifndef BEZIER_LINE_HPP
#define BEZIER_LINE_HPP

#include <SFML/Graphics.hpp>

    const int NUM_SEGMENTS = 20;
class BezierLine
{
public:
    sf::Vector2f start, end, start_control, end_control;

    // bool operator==(const MouthBezierPoints &) const;
    // bool closeEnough(const MouthBezierPoints &, const float) const;
    // MouthBezierPoints transform(const int, const int, const float, const float);
    // void moveTowards(const MouthBezierPoints&, const float, const float);
    std::vector<sf::Vector2f> generateCurve() const;


};

std::vector<sf::Vector2f> BezierLine::generateCurve() const
{

    std::vector<sf::Vector2f> result;

    result.push_back(start);
    float p = 1.f / NUM_SEGMENTS;
    float q = p;
    for (size_t i = 1; i < NUM_SEGMENTS; i++, p += q)
    {
        result.push_back(p * p * p * (end + 3.f * (start_control - end_control) - start) + 3.f * p * p * (start - 2.f * start_control + end_control) + 3.f * p * (start_control - start) + start);
    }
    result.push_back(end);

    return result;
}
/*
bool MouthBezierPoints::operator==(const MouthBezierPoints &other_point) const
{
    if (upper_start == other_point.upper_start &&
        upper_end == other_point.upper_end &&
        upper_start_control == other_point.upper_start_control &&
        upper_end_control == other_point.upper_end_control &&

        lower_start == other_point.lower_start &&
        lower_end == other_point.lower_end &&
        lower_start_control == other_point.lower_start_control &&
        lower_end_control == other_point.lower_end_control)
    {

        return true;
    }
    else
    {
        return false;
    }
}
*/

/*
void MouthBezierPoints::setPoint(const std::string& key, const std::string& x, const std::string& y) {
    if (key == "upper_start") upper_start = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_end") upper_end = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_start_control") upper_start_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "upper_end_control") upper_end_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_start") lower_start = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_end") lower_end = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_start_control") lower_start_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
    if (key == "lower_end_control") lower_end_control = sf::Vector2f(strtof(x.c_str(),0), strtof(y.c_str(),0)); return;
}
*/
/*
bool MouthBezierPoints::closeEnough(const MouthBezierPoints &other_point, const float threshold) const
{

    if (getDistance(upper_start, other_point.upper_start) <= threshold &&
        getDistance(upper_end, other_point.upper_end) <= threshold &&
        getDistance(upper_start_control, other_point.upper_start_control) <= threshold &&
        getDistance(upper_end_control, other_point.upper_end_control) <= threshold &&

        getDistance(lower_start, other_point.lower_start) <= threshold &&
        getDistance(lower_end, other_point.lower_end) <= threshold &&
        getDistance(lower_start_control, other_point.lower_start_control) <= threshold &&
        getDistance(lower_end_control, other_point.lower_end_control) <= threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

MouthBezierPoints MouthBezierPoints::transform(const int offset_x, const int offset_y, const float scale_x, const float scale_y)
{
    return {
        .upper_start = sf::Vector2f(scale_x * upper_start.x + offset_x, scale_y * upper_start.y + offset_y),
        .upper_end = sf::Vector2f(scale_x * upper_end.x + offset_x, scale_y * upper_end.y + offset_y),
        .upper_start_control = sf::Vector2f(scale_x * upper_start_control.x + offset_x, scale_y * upper_start_control.y + offset_y),
        .upper_end_control = sf::Vector2f(scale_x * upper_end_control.x + offset_x, scale_y * upper_end_control.y + offset_y),

        .lower_start = sf::Vector2f(scale_x * lower_start.x + offset_x, scale_y * lower_start.y + offset_y),
        .lower_end = sf::Vector2f(scale_x * lower_end.x + offset_x, scale_y * lower_end.y + offset_y),
        .lower_start_control = sf::Vector2f(scale_x * lower_start_control.x + offset_x, scale_y * lower_start_control.y + offset_y),
        .lower_end_control = sf::Vector2f(scale_x * lower_end_control.x + offset_x, scale_y * lower_end_control.y + offset_y)};
}

void MouthBezierPoints::moveTowards(const MouthBezierPoints &goal_point, const float frame_delta_time, const float speed)
{
    sf::Vector2f direction = normalize(goal_point.upper_start - upper_start);
    upper_start += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.upper_end - upper_end);
    upper_end += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.upper_start_control - upper_start_control);
    upper_start_control += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.upper_end_control - upper_end_control);
    upper_end_control += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.lower_start - lower_start);
    lower_start += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.lower_end - lower_end);
    lower_end += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.lower_start_control - lower_start_control);
    lower_start_control += frame_delta_time * speed * direction * 0.2f;

    direction = normalize(goal_point.lower_end_control - lower_end_control);
    lower_end_control += frame_delta_time * speed * direction * 0.2f;
}
*/

#endif // BEZIER_LINE_HPP
