#ifndef UTILS_H
#define UTILS_H

#include <regex>
#include <SFML/Graphics.hpp>
#include <robot_faces/consts.hpp>

// clamp is available in C++17
template <typename T>
T clamp(const T &n, const T &lower, const T &upper)
{
    return std::max(lower, std::min(n, upper));
}

inline void printTransform(const sf::Transform& transform) {
    const float* matrix = transform.getMatrix();
    std::cout << matrix[0] << ", " << matrix[4] << ", " << matrix[8] << ", " << matrix[12] << std::endl << 
                matrix[1] << ", " << matrix[5] << ", " << matrix[9] << ", " << matrix[13] <<  std::endl << 
                matrix[2] << ", " << matrix[6] << ", " << matrix[10] << ", " << matrix[14] <<  std::endl << 
                matrix[3] << ", " << matrix[7] << ", " << matrix[11] << ", " << matrix[15] << std::endl;
}

sf::Color validateColour(const std::string &c)
{
    if (std::regex_match(c, g_rgba_regex))
    {

        std::stringstream ss(c);

        std::string substr;
        int r = 255, g = 255, b = 255, a = 255;

        if (ss.good())
            std::getline(ss, substr, ',');
        r = std::stoi(substr);

        if (ss.good())
            std::getline(ss, substr, ',');
        g = std::stoi(substr);

        if (ss.good())
            std::getline(ss, substr, ',');
        b = std::stoi(substr);

        if (ss.good())
            std::getline(ss, substr, ',');
        a = std::stoi(substr);

        // colour_ = sf::Color(r, g, b, a);
        return sf::Color(r, g, b, a);
    }
    else
    {
        ROS_ERROR("colour parameter is not formatted incorrectly.");
        throw "Colour not formatted";
    }
}

inline float degToRad(const float deg)
{
    return deg * M_PI / 180.0f;
}
inline float getMagnitude(const sf::Vector2f v)
{
    return (float)sqrt(v.x * v.x + v.y * v.y);
}
inline sf::Vector2f normalize(const sf::Vector2f v)
{
    float mag = getMagnitude(v);
    return mag != 0 ? v / mag : sf::Vector2f(0, 0);
}

// inline float getDistance(const sf::Vector2f one, const sf::Vector2f two) {
//  return sqrt(pow(two.x - one.x, 2) + pow(two.y - one.y, 2));
// }

void readVerticesFromFile(sf::VertexArray &vertex_array, std::string file_path, sf::Color colour)
{

    vertex_array.clear();

    sf::Vector2f initial_position{0, 0}; //TODO REMOVE
    std::string package_path = ros::package::getPath("robot_faces");

    std::ifstream file(package_path + file_path, std::ifstream::in);
    std::string line, x, y;
    if (!getline(file, line))
    {
        ROS_ERROR("Could not open file");
    }
    std::stringstream liness(line);

    vertex_array.append(sf::Vertex(sf::Vector2f(initial_position.x, initial_position.y), colour));

    while (getline(file, line))
    {
        std::stringstream liness(line);
        getline(liness, x, ',');
        getline(liness, y);

        float x_point = strtof(x.c_str(), 0);
        float y_point = strtof(y.c_str(), 0);

        x_point = (float)initial_position.x + x_point;
        y_point = (float)initial_position.y + y_point;
        vertex_array.append(sf::Vertex(sf::Vector2f(x_point, y_point), colour));
    }
    file.close();
}

sf::VertexArray generateLineWithThickness(const std::vector<sf::Vector2f> &points, const sf::Color color, const float thickness)
{
    sf::VertexArray array = sf::VertexArray(sf::TrianglesStrip);

    for (int i = 0; i < points.size() - 1; i++)
    {
        sf::Vector2f line = points[i] - points[i + 1];
        sf::Vector2f normal = normalize(sf::Vector2f(-line.y, line.x));

        array.append(sf::Vertex(points[i] - thickness * normal, color));
        array.append(sf::Vertex(points[i] + thickness * normal, color));

        array.append(sf::Vertex(points[i + 1] - thickness * normal, color));
        array.append(sf::Vertex(points[i + 1] + thickness * normal, color));
    }

    return array;
}

#endif // UTILS_H