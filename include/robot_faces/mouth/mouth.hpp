#ifndef MOUTH_HPP
#define MOUTH_HPP

#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <robot_faces/entities/proxy-entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/mouth/line-mouth.hpp>
#include <robot_faces/mouth/squircle-mouth.hpp>

const std::multimap<MouthShape, std::shared_ptr<Entity>> MOUTH_ENTITIES_{
    {MouthShape::Squircle, std::make_shared<SquircleMouth>()},
    {MouthShape::Line, std::make_shared<LineMouth>()}};

class Mouth : public ProxyEntity<MouthShape>
{

public:
    Mouth() : ProxyEntity(MouthShape::Line, MOUTH_ENTITIES_) {}

    void setSpeaking(const bool speaking)
    {
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

            }
        }
    }

    void setSquircleRadius(const sf::Vector2f squircle_radius)
    {
        for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(MouthShape::Squircle)); range.first != range.second; ++range.first)
        {
            std::shared_ptr<SquircleMouth> squircle_cast = std::static_pointer_cast<SquircleMouth>(range.first->second);
            squircle_cast->setSquircleRadius(squircle_radius);
        }
    }
};

#endif // MOUTH_HPP