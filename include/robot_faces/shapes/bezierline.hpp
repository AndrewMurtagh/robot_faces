#ifndef BEZIER_LINE_HPP
#define BEZIER_LINE_HPP

#include <SFML/Graphics.hpp>

const int NUM_SEGMENTS = 20;
class BezierLine
{
public:
    sf::Vector2f start, end, start_control, end_control;
    
    std::vector<sf::Vector2f> generateCurve() const
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
};

#endif // BEZIER_LINE_HPP
