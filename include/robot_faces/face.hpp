#ifndef FACE_H
#define FACE_H

#include <ros/ros.h>

#include <robot_faces/consts.hpp>
#include <robot_faces/face-config.hpp>
#include <robot_faces/entities/proxy-entity.hpp>
#include <robot_faces/entities/vertex-entity.hpp>
#include <robot_faces/eyes/pupil.hpp>
#include <robot_faces/eyes/iris.hpp>
#include <robot_faces/nose/nose.hpp>
#include <robot_faces/mouth/mouth.hpp>

const std::multimap<EyebrowShape, std::shared_ptr<Entity>> LEFT_EYEBROW_ENTITIES{
    {EyebrowShape::Rectangle, std::make_shared<VertexEntity>("/res/eyebrow/rectangle.txt")},
    {EyebrowShape::Square, std::make_shared<VertexEntity>("/res/eyebrow/square.txt")},
    {EyebrowShape::Rounded, std::make_shared<VertexEntity>("/res/eyebrow/rounded.txt")},
    {EyebrowShape::Straight, std::make_shared<VertexEntity>("/res/eyebrow/straight.txt")},
    {EyebrowShape::Arch, std::make_shared<VertexEntity>("/res/eyebrow/arch.txt")},
    {EyebrowShape::Arc, std::make_shared<VertexEntity>("/res/eyebrow/arc.txt")}};

const std::multimap<EyebrowShape, std::shared_ptr<Entity>> RIGHT_EYEBROW_ENTITIES{
    {EyebrowShape::Rectangle, std::make_shared<VertexEntity>("/res/eyebrow/rectangle.txt")},
    {EyebrowShape::Square, std::make_shared<VertexEntity>("/res/eyebrow/square.txt")},
    {EyebrowShape::Rounded, std::make_shared<VertexEntity>("/res/eyebrow/rounded.txt")},
    {EyebrowShape::Straight, std::make_shared<VertexEntity>("/res/eyebrow/straight.txt")},
    {EyebrowShape::Arch, std::make_shared<VertexEntity>("/res/eyebrow/arch.txt")},
    {EyebrowShape::Arc, std::make_shared<VertexEntity>("/res/eyebrow/arc.txt")}};

class Face
{

    bool DEBUG_ONCE;
    float DEBUG_TIME;

public:
    Face() : left_eyebrow_(EyebrowShape::Arc, LEFT_EYEBROW_ENTITIES),
             right_eyebrow_(EyebrowShape::Arc, RIGHT_EYEBROW_ENTITIES)
    {
        ROS_INFO("Face::Face");
        DEBUG_TIME = 0.0f;
        DEBUG_ONCE = false;
        // DEBUG_MARKER.setRadius(REF_MARKER_RADIUS);
        // DEBUG_MARKER.setOrigin(REF_MARKER_RADIUS, REF_MARKER_RADIUS);
        // DEBUG_MARKER.setFillColor(REF_MARKER_COLOUR);
        // DEBUG_MARKER_2.setRadius(REF_MARKER_RADIUS);
        // DEBUG_MARKER_2.setOrigin(REF_MARKER_RADIUS, REF_MARKER_RADIUS);
        // DEBUG_MARKER_2.setFillColor(REF_MARKER_COLOUR);
    }

    void setGaze();

    void setExpression();

    void setPreset();

    void configure(const FaceConfiguration &face_config);

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
    {
        // ROS_INFO("Face::draw");

        DEBUG_TIME += frame_delta_time;

        if (DEBUG_TIME > 3.0f && !DEBUG_ONCE)
        {
            // nose_.setShape(NoseShape::Curve);
            mouth_.setSpeaking(true);
            DEBUG_ONCE = true;
        }

        left_iris_.draw(renderWindow, frame_delta_time);
        right_iris_.draw(renderWindow, frame_delta_time);
        left_pupil_.draw(renderWindow, frame_delta_time);
        right_pupil_.draw(renderWindow, frame_delta_time);
        left_eyebrow_.draw(renderWindow, frame_delta_time);
        right_eyebrow_.draw(renderWindow, frame_delta_time);
        mouth_.draw(renderWindow, frame_delta_time);
        nose_.draw(renderWindow, frame_delta_time);

        // renderWindow.draw(DEBUG_MARKER_2, DEBUG_LEFT_EYE_TRANSFORM);
        // renderWindow.draw(DEBUG_MARKER, DEBUG_RIGHT_EYE_TRANSFORM);
    }

private:
    Nose nose_;
    Mouth mouth_;
    Iris left_iris_;
    Iris right_iris_;
    Pupil left_pupil_;
    Pupil right_pupil_;
    // sf::CircleShape DEBUG_MARKER;
    // sf::CircleShape DEBUG_MARKER_2;
    // sf::Transform DEBUG_LEFT_EYE_TRANSFORM;
    // sf::Transform DEBUG_RIGHT_EYE_TRANSFORM;
    ProxyEntity<EyebrowShape> left_eyebrow_;
    ProxyEntity<EyebrowShape> right_eyebrow_;
};

void Face::configure(const FaceConfiguration &face_config)
{
    ROS_INFO("Face::configure");

    /*
    Misc
    */
    nose_.setBackgroundColour(face_config.background_colour);


    const float WINDOW_WIDTH = face_config.window_width;
    const float WINDOW_HEIGHT = face_config.window_height;

    sf::Transform face_center_transform(1.0f, 0.0f, WINDOW_WIDTH * 0.5f,
                                        0.0f, 1.0f, WINDOW_HEIGHT * face_config.face_center,
                                        0.0f, 0.0f, 1.f);

    sf::Transform left_eye_face_transform;
    left_eye_face_transform.scale(face_config.pupil_scaling);
    left_eye_face_transform.combine(face_center_transform);
    left_eye_face_transform.translate(-0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);

    sf::Transform right_eye_face_transform;
    right_eye_face_transform.scale(face_config.pupil_scaling);
    right_eye_face_transform.combine(face_center_transform);
    right_eye_face_transform.translate(0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);
    right_eye_face_transform.combine(g_mirror_transform);

    sf::Transform left_eyebrow_eye_transform;
    left_eyebrow_eye_transform.scale(face_config.eyebrow_scaling);
    left_eyebrow_eye_transform.combine(left_eye_face_transform);
    left_eyebrow_eye_transform.translate(0, face_config.eyebrow_spacing * WINDOW_HEIGHT);

    sf::Transform right_eyebrow_eye_transform;
    right_eyebrow_eye_transform.scale(face_config.eyebrow_scaling);
    right_eyebrow_eye_transform.combine(right_eye_face_transform);
    right_eyebrow_eye_transform.translate(0, face_config.eyebrow_spacing * WINDOW_HEIGHT);
    right_eyebrow_eye_transform.combine(g_mirror_transform);

    sf::Transform nose_face_transform;
    nose_face_transform.combine(face_center_transform);
    nose_face_transform.translate(0, WINDOW_HEIGHT * face_config.nose_y);
    nose_face_transform.scale(face_config.nose_scaling);

    sf::Transform mouth_face_transform;
    mouth_face_transform.scale(face_config.mouth_scaling);
    mouth_face_transform.translate(0, WINDOW_HEIGHT * face_config.mouth_y);


    /*
    Eyes
    */
    int avg_blink_interval;
    bool should_blink;
    bool should_do_saccades;
    // float eye_spacing;
    // float eye_y;

    // bool show_pupil_highlights;
    sf::Vector2f pupil_squircle_radius; // not working
    sf::Vector2f pupil_scaling;
    // sf::Color pupil_colour;
    // bool show_pupil;

    IrisShape iris_shape;
    sf::Vector2f iris_squircle_radius;
    sf::Vector2f iris_scaling;
    sf::Color iris_colour;
    bool show_iris;

    
    // iris
    left_iris_.setShape(face_config.iris_shape);
    left_iris_.setTransformation(left_eye_face_transform);
    left_iris_.setColour(face_config.iris_colour);
    left_iris_.setSquircleRadius(face_config.iris_squircle_radius);
    left_iris_.show(face_config.show_iris);

    right_iris_.setShape(face_config.iris_shape);
    right_iris_.setTransformation(right_eye_face_transform);
    right_iris_.setColour(face_config.iris_colour);
    right_iris_.setSquircleRadius(face_config.iris_squircle_radius);
    right_iris_.show(face_config.show_iris);
    
    // left pupil
    left_pupil_.setTransformation(left_eye_face_transform);
    left_pupil_.setColour(face_config.pupil_colour);
    left_pupil_.setSquircleRadius(face_config.pupil_squircle_radius);
    left_pupil_.setPupilHighlightShow(face_config.show_pupil_highlights);
    // left_pupil_.show(face_config.show_pupil);

    // right pupil
    right_pupil_.setTransformation(right_eye_face_transform);
    right_pupil_.setColour(face_config.pupil_colour);
    right_pupil_.setSquircleRadius(face_config.pupil_squircle_radius);
    right_pupil_.setPupilHighlightShow(face_config.show_pupil_highlights);
    // right_pupil_.show(face_config.show_pupil);
    

    /*
    Eyebrows
    */
    left_eyebrow_.setShape(face_config.eyebrow_shape);
    left_eyebrow_.setTransformation(left_eyebrow_eye_transform);
    left_eyebrow_.setColour(face_config.eyebrow_colour);
    left_eyebrow_.show(face_config.show_eyebrows);

    right_eyebrow_.setShape(face_config.eyebrow_shape);
    right_eyebrow_.setTransformation(right_eyebrow_eye_transform);
    right_eyebrow_.setColour(face_config.eyebrow_colour);
    right_eyebrow_.show(face_config.show_eyebrows);

    /*
    Nose
    */
    nose_.setShape(face_config.nose_shape);
    nose_.setTransformation(nose_face_transform);
    nose_.setColour(face_config.nose_colour);
    nose_.setSquircleRadius(face_config.nose_squircle_radius);
    nose_.show(face_config.show_nose);

    /*
    Mouth
    */
    mouth_.setShape(face_config.mouth_shape);
    mouth_.setTransformation(face_center_transform * mouth_face_transform);
    mouth_.setColour(face_config.mouth_colour);
    mouth_.setSquircleRadius(face_config.mouth_squircle_radius);
    mouth_.show(face_config.show_mouth);

}

#endif // FACE_H
