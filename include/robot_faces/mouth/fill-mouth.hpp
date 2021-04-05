#ifndef FILL_MOUTH_HPP
#define FILL_MOUTH_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/utils.hpp>
#include <robot_faces/mouth/line-mouth.hpp>
#include <robot_faces/shapes/bezierline.hpp>

class FillMouth : public LineMouth
{
public:
    void draw(sf::RenderWindow &, const float) override;
};

void FillMouth::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{

    LineMouth::draw(renderWindow, frame_delta_time);

    sf::VertexArray filled_vertex_array = sf::VertexArray(sf::TrianglesStrip);

    for (int i = 0; i < upper_bezier_line_points_.size() - 1; i++)
    {
        filled_vertex_array.append(sf::Vertex(upper_bezier_line_points_[i], colour_));
        filled_vertex_array.append(sf::Vertex(lower_bezier_line_points_[i], colour_));
    }
    renderWindow.draw(filled_vertex_array, transform_);

}

#endif // FILL_MOUTH_HPP