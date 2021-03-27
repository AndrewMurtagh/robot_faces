#ifndef SQUIRCLE_SHAPE_HPP
#define SQUIRCLE_SHAPE_HPP

/*
modified from:
https://github.com/SFML/SFML/wiki/Source%3A-Draw-Rounded-Rectangle
*/

#include <SFML/Graphics/Shape.hpp>

namespace sf
{

    class SquircleShape : public sf::Shape
    {
    public:
        SquircleShape(
            const Vector2f &size = Vector2f(0, 0),
            const Vector2f &radius = Vector2f(0, 0),
            unsigned int cornerPointCount = 0)
        {
            mySize = size;
            radius_ = radius;
            myRadius = 10;
            myCornerPointCount = cornerPointCount;
            update();
        }

        void setSize(const Vector2f &size)
        {
            mySize = size;
            update();
        }

        const Vector2f &getSize() const
        {
            return mySize;
        }

        void setRadius(const Vector2f radius)
        {
            radius_ = radius;
            update();
        }

        Vector2f setRadius() const
        {
            return radius_;
        }

        void setCornerPointCount(unsigned int count)
        {
            myCornerPointCount = count;
            update();
        }

        virtual std::size_t getPointCount() const
        {
            return myCornerPointCount * 4;
        }

        virtual sf::Vector2f getPoint(std::size_t index) const
        {

            if (index >= myCornerPointCount * 4)
                return sf::Vector2f(0, 0);

            float deltaAngle = 90.0f / (myCornerPointCount - 1);
            sf::Vector2f center;
            unsigned int centerIndex = index / myCornerPointCount;
            static const float pi = 3.141592654f;

            switch (centerIndex)
            {
            case 0:
                center.x = mySize.x - radius_.x;
                center.y = radius_.y;
                break;
            case 1:
                center.x = radius_.x;
                center.y = radius_.y;
                break;
            case 2:
                center.x = radius_.x;
                center.y = mySize.y - radius_.y;
                break;
            case 3:
                center.x = mySize.x - radius_.x;
                center.y = mySize.y - radius_.y;
                break;
            }

            return sf::Vector2f(radius_.x * cos(deltaAngle * (index - centerIndex) * pi / 180) + center.x, 
            -radius_.y * sin(deltaAngle * (index - centerIndex) * pi / 180) + center.y);
        }

    private:
        Vector2f mySize;
        Vector2f radius_;
        float myRadius;
        unsigned int myCornerPointCount;
    };
}
#endif // SQUIRCLE_SHAPE_HPP
