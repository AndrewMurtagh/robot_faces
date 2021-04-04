#ifndef NOSE_HPP
#define NOSE_HPP

#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <robot_faces/entities/proxy-entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/nose/annulus-nose.hpp>
#include <robot_faces/nose/curve-nose.hpp>
#include <robot_faces/entities/vertex-entity.hpp>
#include <robot_faces/entities/squircle-entity.hpp>

const std::multimap<NoseShape, std::shared_ptr<Entity>> NOSE_ENTITIES_{
    {NoseShape::Annulus, std::make_shared<AnnulusNose>()},
    {NoseShape::Curve, std::make_shared<CurveNose>()},
    {NoseShape::Animal, std::make_shared<VertexEntity>("/res/nose/animal.txt")},
    {NoseShape::Squircle, std::make_shared<SquircleEntity>(NOSE_SIZE, SQUIRCLE_NOSE_RADIUS)}};

class Nose : public ProxyEntity<NoseShape>
{

public:
    Nose();

    void setBackgroundColour(const sf::Color);

    void setSquircleRadius(const sf::Vector2f);
};

Nose::Nose() : ProxyEntity(NoseShape::Squircle, NOSE_ENTITIES_) {}

void Nose::setBackgroundColour(const sf::Color background_colour)
{
    for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(NoseShape::Annulus)); range.first != range.second; ++range.first)
    {
        std::shared_ptr<AnnulusNose> annulus_cast = std::static_pointer_cast<AnnulusNose>(range.first->second);
        annulus_cast->setBackgroundColour(background_colour);
    }
}

void Nose::setSquircleRadius(const sf::Vector2f squircle_radius)
{
    for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(NoseShape::Squircle)); range.first != range.second; ++range.first)
    {
        std::shared_ptr<SquircleEntity> squircle_cast = std::static_pointer_cast<SquircleEntity>(range.first->second);
        squircle_cast->setSquircleRadius(squircle_radius);
    }
}

#endif // NOSE_HPP