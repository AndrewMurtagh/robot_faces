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

const std::multimap<EyebrowShape, std::shared_ptr<Entity>> EYEBROW_ENTITIES{
    {EyebrowShape::Rectangle, std::make_shared<VertexEntity>("/res/eyebrow/rectangle.txt")},
    {EyebrowShape::Square, std::make_shared<VertexEntity>("/res/eyebrow/square.txt")},
    {EyebrowShape::Rounded, std::make_shared<VertexEntity>("/res/eyebrow/rounded.txt")},
    {EyebrowShape::Straight, std::make_shared<VertexEntity>("/res/eyebrow/straight.txt")},
    {EyebrowShape::Arch, std::make_shared<VertexEntity>("/res/eyebrow/arch.txt")},
    {EyebrowShape::Arc, std::make_shared<VertexEntity>("/res/eyebrow/arc.txt")}};

class Face
{

public:
    Face() : left_eyebrow_(EyebrowShape::Arc, EYEBROW_ENTITIES),
             right_eyebrow_(EyebrowShape::Arc, EYEBROW_ENTITIES),
             background_colour_(sf::Color(255, 255, 255, 255))
    {
        DEBUG_ONCE = true;
        DEBUG_TIMER = 0.0f;

        DEBUG_GAZE_MARKER.setRadius(5);
        DEBUG_GAZE_MARKER.setOrigin(5, 5);
        DEBUG_GAZE_MARKER.setFillColor(sf::Color(255, 0, 255, 255));
    }

    void setSpeaking(const bool speaking)
    {
        mouth_.setSpeaking(speaking);
    }

    void setGaze(const sf::Vector2f gaze_vector)
    {
        // std::cout << gaze_vector.x << " , " << gaze_vector.y << std::endl;
        const sf::Vector2f delta = sf::Vector2f(gaze_vector.x * MAX_GAZE_SIZE.x, gaze_vector.y * MAX_GAZE_SIZE.y);

        sf::Transform gaze_transform = left_iris_.getTransformation();
        gaze_transform.translate(delta*PUPIL_IRIS_DELTA);
        left_iris_.setTransformation(gaze_transform);

        gaze_transform = right_iris_.getTransformation();
        gaze_transform.translate(delta*PUPIL_IRIS_DELTA);
        right_iris_.setTransformation(gaze_transform);

        gaze_transform = left_pupil_.getTransformation();
        gaze_transform.translate(delta);
        left_pupil_.setTransformation(gaze_transform);

        gaze_transform = right_pupil_.getTransformation();
        gaze_transform.translate(delta);
        right_pupil_.setTransformation(gaze_transform);
    }

    void setExpression();

    void setPreset();

    void configure(const FaceConfiguration &face_config);

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
    {
        DEBUG_TIMER += frame_delta_time;
        if (DEBUG_TIMER > 2000 && DEBUG_ONCE)
        {
            DEBUG_ONCE = false;
            std::cout << "move mouth" << std::endl;

            sf::Transform face_center_transform(1.0f, 0.0f, 800 * 0.75f,
                                                0.0f, 1.0f, 600 * 0.5f,
                                                0.0f, 0.0f, 1.f);
            // mouth_.setTransformation(face_center_transform);
        }

        renderWindow.clear(background_colour_);

        left_iris_.draw(renderWindow, frame_delta_time);
        right_iris_.draw(renderWindow, frame_delta_time);

        left_pupil_.draw(renderWindow, frame_delta_time);
        right_pupil_.draw(renderWindow, frame_delta_time);

        left_eyebrow_.draw(renderWindow, frame_delta_time);
        right_eyebrow_.draw(renderWindow, frame_delta_time);

        mouth_.draw(renderWindow, frame_delta_time);

        nose_.draw(renderWindow, frame_delta_time);

        // renderWindow.draw(DEBUG_GAZE_MARKER, left_eye_face_transform);
    }

private:
    ProxyEntity<EyebrowShape> left_eyebrow_;
    ProxyEntity<EyebrowShape> right_eyebrow_;
    Iris left_iris_;
    Iris right_iris_;
    Pupil left_pupil_;
    Pupil right_pupil_;
    Nose nose_;
    Mouth mouth_;
    sf::Color background_colour_;

    bool DEBUG_ONCE;
    float DEBUG_TIMER;
    sf::CircleShape DEBUG_GAZE_MARKER;
    sf::Transform DEBUG_GAZE_TRANSFORM;
    sf::Transform DEBUG_LEFT_EYE_TRANSFORM;
};

void Face::configure(const FaceConfiguration &face_config)
{

    /*
    Misc
    */
    background_colour_ = face_config.background_colour;
    nose_.setBackgroundColour(face_config.background_colour);

    const float WINDOW_WIDTH = face_config.window_width;
    const float WINDOW_HEIGHT = face_config.window_height;

    sf::Transform face_center_transform(1.0f, 0.0f, WINDOW_WIDTH * 0.5f,
                                        0.0f, 1.0f, WINDOW_HEIGHT * face_config.face_center,
                                        0.0f, 0.0f, 1.f);

    sf::Transform left_eye_face_transform;
    left_eye_face_transform = sf::Transform::Identity;
    left_eye_face_transform.combine(face_center_transform);
    left_eye_face_transform.translate(-0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);
    left_eye_face_transform.scale(face_config.pupil_scaling);
    // DEBUG_LEFT_EYE_TRANSFORM = left_eye_face_transform;

    sf::Transform right_eye_face_transform;
    right_eye_face_transform = sf::Transform::Identity;
    right_eye_face_transform.combine(face_center_transform);
    right_eye_face_transform.translate(0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);
    right_eye_face_transform.scale(face_config.pupil_scaling);
    // right_eye_face_transform.combine(g_mirror_transform);

    sf::Transform left_eyebrow_eye_transform;
    left_eyebrow_eye_transform.combine(left_eye_face_transform);
    left_eyebrow_eye_transform.translate(0, face_config.eyebrow_spacing * WINDOW_HEIGHT);
    left_eyebrow_eye_transform.scale(face_config.eyebrow_scaling);

    sf::Transform right_eyebrow_eye_transform;
    right_eyebrow_eye_transform.combine(right_eye_face_transform);
    right_eyebrow_eye_transform.translate(0, face_config.eyebrow_spacing * WINDOW_HEIGHT);
    right_eyebrow_eye_transform.scale(face_config.eyebrow_scaling);

    sf::Transform nose_face_transform;
    nose_face_transform.combine(face_center_transform);
    nose_face_transform.translate(0, WINDOW_HEIGHT * face_config.nose_y);
    nose_face_transform.scale(face_config.nose_scaling);

    sf::Transform mouth_face_transform;
    mouth_face_transform.combine(face_center_transform);
    mouth_face_transform.translate(0, WINDOW_HEIGHT * face_config.mouth_y);
    mouth_face_transform.scale(face_config.mouth_scaling);

    /*
    Eyes
    */

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

    // right pupil
    right_pupil_.setTransformation(right_eye_face_transform);
    right_pupil_.setColour(face_config.pupil_colour);
    right_pupil_.setSquircleRadius(face_config.pupil_squircle_radius);
    right_pupil_.setPupilHighlightShow(face_config.show_pupil_highlights);

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
    mouth_.setTransformation(mouth_face_transform);
    mouth_.setColour(face_config.mouth_colour);
    mouth_.setSquircleRadius(face_config.mouth_squircle_radius);
    mouth_.show(face_config.show_mouth);
}

#endif // FACE_H
