#ifndef IRIS_HPP
#define IRIS_HPP

#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/entities/proxy-entity.hpp>
#include <robot_faces/entities/squircle-entity.hpp>

const std::multimap<IrisShape, std::shared_ptr<Entity>> IRIS_ENTITIES_{
    {IrisShape::Thick, std::make_shared<VertexEntity>("/res/iris/thick.txt")},
    {IrisShape::Oval, std::make_shared<VertexEntity>("/res/iris/oval.txt")},
    {IrisShape::Almond, std::make_shared<VertexEntity>("/res/iris/almond.txt")},
    {IrisShape::Arc, std::make_shared<VertexEntity>("/res/iris/arc.txt")},
    {IrisShape::Squircle, std::make_shared<SquircleEntity>(IRIS_SIZE, SQUIRCLE_IRIS_RADIUS)}};


class Iris : public ProxyEntity<IrisShape>
{

public:
    Iris() : ProxyEntity(IrisShape::Squircle, IRIS_ENTITIES_) {}

    void setSquircleRadius(const sf::Vector2f squircle_radius)
    {
        ROS_INFO("Iris::setSquircleRadius");
        for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(IrisShape::Squircle)); range.first != range.second; ++range.first)
        {
            std::shared_ptr<SquircleEntity> squircle_cast = std::static_pointer_cast<SquircleEntity>(range.first->second);
            squircle_cast->setSquircleRadius(squircle_radius);
        }
    }

};

#endif // IRIS_HPP