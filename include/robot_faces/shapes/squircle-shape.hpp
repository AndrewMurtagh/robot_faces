#ifndef SQUIRCLE_SHAPE_HPP
#define SQUIRCLE_SHAPE_HPP

/*
modified from:
https://github.com/SFML/SFML/wiki/Source%3A-Draw-Rounded-Rectangle
*/

#include <SFML/Graphics/Shape.hpp>
#include <robot_faces/consts.hpp>

namespace sf
{

    class SquircleShape : public sf::Shape
    {
    public:
        SquircleShape(const Vector2f &, const Vector2f &, unsigned int);

        void setSize(const Vector2f &);

        const Vector2f &getSize() const;

        void setRadius(const Vector2f);

        virtual std::size_t getPointCount() const;

        virtual sf::Vector2f getPoint(std::size_t) const;

    private:
        Vector2f size_;
        Vector2f radius_;
    };
}

sf::SquircleShape::SquircleShape(
    const Vector2f &size = Vector2f(0, 0),
    const Vector2f &radius = Vector2f(0, 0),
    unsigned int cornerPointCount = 0)
{
    size_ = size;
    radius_ = radius;
    update();
}

void sf::SquircleShape::setSize(const Vector2f &size)
{
    size_ = size;
    update();
}

const sf::Vector2f &sf::SquircleShape::getSize() const
{
    return size_;
}

void sf::SquircleShape::setRadius(const Vector2f radius)
{
    radius_ = radius;
    update();
}

std::size_t sf::SquircleShape::getPointCount() const
{
    return NUM_CORNER_POINTS * 4;
}

sf::Vector2f sf::SquircleShape::getPoint(std::size_t index) const
{

    if (index >= NUM_CORNER_POINTS * 4)
        return sf::Vector2f(0, 0);

    float delta_angle = 90.0f / (NUM_CORNER_POINTS - 1);
    sf::Vector2f center;
    unsigned int center_index = index / NUM_CORNER_POINTS;
    static const float pi = 3.141592654f;

    switch (center_index)
    {
    case 0:
        center.x = size_.x - radius_.x;
        center.y = radius_.y;
        break;
    case 1:
        center.x = radius_.x;
        center.y = radius_.y;
        break;
    case 2:
        center.x = radius_.x;
        center.y = size_.y - radius_.y;
        break;
    case 3:
        center.x = size_.x - radius_.x;
        center.y = size_.y - radius_.y;
        break;
    }

    return sf::Vector2f(radius_.x * cos(delta_angle * (index - center_index) * pi / 180) + center.x,
                        -radius_.y * sin(delta_angle * (index - center_index) * pi / 180) + center.y);
}

#endif // SQUIRCLE_SHAPE_HPP
