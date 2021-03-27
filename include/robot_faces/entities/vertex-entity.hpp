#ifndef VERTEX_ENTITY_H
#define VERTEX_ENTITY_H

#include <SFML/Graphics.hpp>
#include <robot_faces/entities/ientity.hpp>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/utils.hpp>


class VertexEntity : public Entity
{
public:
    VertexEntity(const std::string& filename) : vertex_points_(sf::TrianglesFan)
    {
        readVerticesFromFile(vertex_points_, filename, colour_);
    }

    void setColour(const sf::Color colour) override
    {
        Entity::setColour(colour);
        for(std::size_t i=0; i< vertex_points_.getVertexCount(); ++i) {
            vertex_points_[i].color = colour;
        }
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        renderWindow.draw(vertex_points_, transform_);
        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    sf::VertexArray vertex_points_;
};



#endif // VERTEX_ENTITY_H