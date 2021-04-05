#ifndef BEZIER_LINE_HPP
#define BEZIER_LINE_HPP

#include <SFML/Graphics.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/utils.hpp>

class BezierLine
{
public:
    sf::Vector2f start, end, start_control, end_control;

    std::vector<sf::Vector2f> generateCurve() const;
    bool closeEnough(const BezierLine &other_point, const float threshold) const;
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

bool BezierLine::closeEnough(const BezierLine &other_point, const float threshold) const
{

    if (getDistance(start, other_point.start) <= threshold &&
        getDistance(end, other_point.end) <= threshold &&
        getDistance(start_control, other_point.start_control) <= threshold &&
        getDistance(end_control, other_point.end_control) <= threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}



void readBezierPointsFromFile(BezierLine &upper_points, BezierLine &lower_points, std::string file_path)
{
    std::string package_path = ros::package::getPath("robot_faces");

    std::ifstream file(package_path + file_path, std::ifstream::in);

    if (file.fail())
    {
        ROS_ERROR("Could not open file");
    }

    std::string line, x, y;
    std::stringstream line_stream;

    // upper start
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    upper_points.start = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // upper start control
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    upper_points.start_control = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // upper end control
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    upper_points.end_control = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // upper end
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    upper_points.end = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // lower start control
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    lower_points.start = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // lower start control
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    lower_points.start_control = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // lower end control
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    lower_points.end_control = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    // lower end
    getline(file, line);
    line_stream = std::stringstream(line);
    getline(line_stream, x, ',');
    getline(line_stream, y);
    lower_points.end = sf::Vector2f(strtof(x.c_str(), 0) * MOUTH_SIZE.x, strtof(y.c_str(), 0) * MOUTH_SIZE.y);

    file.close();
}



#endif // BEZIER_LINE_HPP
