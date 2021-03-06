#ifndef MOUTH_HPP
#define MOUTH_HPP

#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <robot_faces/entities/proxy-entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/mouth/line-mouth.hpp>
#include <robot_faces/mouth/squircle-mouth.hpp>
#include <robot_faces/mouth/fill-mouth.hpp>

const std::multimap<MouthShape, std::shared_ptr<Entity>> MOUTH_ENTITIES_{
    {MouthShape::Squircle, std::make_shared<SquircleMouth>()},
    {MouthShape::Line, std::make_shared<LineMouth>()},
    {MouthShape::Fill, std::make_shared<FillMouth>()}};

class Mouth : public ProxyEntity<MouthShape>
{

public:
    Mouth();

    void setSpeaking(const bool);

    void setSquircleRadius(const sf::Vector2f);

    void setExpression(const Expression expression) override
    {
        // std::cout << "Mouth::setExpression" << std::endl;
        // TODO: FIND A BETTER WAY TO DO THIS
        for (EntityMapPair entity : entity_map_)
        {
            switch (entity.first)
            {

            // case MouthShape::Squircle:
            // {
            //     std::shared_ptr<SquircleMouth> squircle_cast = std::static_pointer_cast<SquircleMouth>(entity.second);
            //     squircle_cast->setSpeaking(speaking);
            //     break;
            // }

            case MouthShape::Line:
            {
                std::shared_ptr<LineMouth> line_cast = std::static_pointer_cast<LineMouth>(entity.second);
                line_cast->setExpression(expression);
                break;
            }
            case MouthShape::Fill:
            {
                std::shared_ptr<FillMouth> fill_cast = std::static_pointer_cast<FillMouth>(entity.second);
                fill_cast->setExpression(expression);
                break;
            }
            }
        }
    }
};

Mouth::Mouth() : ProxyEntity(MouthShape::Fill, MOUTH_ENTITIES_) {}

void Mouth::setSpeaking(const bool speaking)
{
    // TODO: FIND A BETTER WAY TO DO THIS
    for (EntityMapPair entity : entity_map_)
    {
        switch (entity.first)
        {

        case MouthShape::Squircle:
        {
            std::shared_ptr<SquircleMouth> squircle_cast = std::static_pointer_cast<SquircleMouth>(entity.second);
            squircle_cast->setSpeaking(speaking);
            break;
        }

        case MouthShape::Line:
        {
            std::shared_ptr<LineMouth> line_cast = std::static_pointer_cast<LineMouth>(entity.second);
            line_cast->setSpeaking(speaking);
            break;
        }
        case MouthShape::Fill:
        {
            std::shared_ptr<FillMouth> fill_cast = std::static_pointer_cast<FillMouth>(entity.second);
            fill_cast->setSpeaking(speaking);
            break;
        }
        }
    }
}

void Mouth::setSquircleRadius(const sf::Vector2f squircle_radius)
{
    for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(MouthShape::Squircle)); range.first != range.second; ++range.first)
    {
        std::shared_ptr<SquircleMouth> squircle_cast = std::static_pointer_cast<SquircleMouth>(range.first->second);
        squircle_cast->setSquircleRadius(squircle_radius);
    }
}

#endif // MOUTH_HPP