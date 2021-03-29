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

struct TransformationTree
{

    TransformationTree()
    {
        DEBUG_GAZE_MARKER.setRadius(5);
        DEBUG_GAZE_MARKER.setOrigin(5, 5);
        DEBUG_GAZE_MARKER.setFillColor(sf::Color(255, 0, 255, 255));
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
    {
        renderWindow.draw(DEBUG_GAZE_MARKER, left_eye_transform_);
        renderWindow.draw(DEBUG_GAZE_MARKER, right_eye_transform_);
    }

    sf::Transform left_eye_transform_;
    sf::Transform right_eye_transform_;
    sf::Vector2f gaze_offset_vec;
    sf::Vector2f saccade_offset_vec;
    sf::CircleShape DEBUG_GAZE_MARKER;
};

class Face
{

public:
    Face() : left_eyebrow_(EyebrowShape::Arc, EYEBROW_ENTITIES),
             right_eyebrow_(EyebrowShape::Arc, EYEBROW_ENTITIES),
             background_colour_(sf::Color(255, 255, 255, 255)),
             saccade_time_dist_(MEAN_SACCADE_TIME, STDDEV_SACCADE_TIME),
             saccade_pos_dist_(0.0f, STDDEV_SACCADE_POS),
             do_saccades_(true)
    {
        DEBUG_ONCE = true;
        DEBUG_TIMER = 0.0f;

        std::random_device rd;
        random_engine_.seed(rd());

        saccade_countdown_ = saccade_time_dist_(random_engine_);
    }

    void setSpeaking(const bool speaking)
    {
        mouth_.setSpeaking(speaking);
    }

    void setGaze(const sf::Vector2f gaze_vector)
    {
        // std::cout << gaze_vector.x << " , " << gaze_vector.y << std::endl;

        transformation_tree_.gaze_offset_vec = sf::Vector2f(gaze_vector.x * MAX_GAZE_SIZE.x, gaze_vector.y * MAX_GAZE_SIZE.y);

        sf::Transform temp_transform = transformation_tree_.left_eye_transform_;
        temp_transform.translate(transformation_tree_.gaze_offset_vec * PUPIL_IRIS_DELTA).translate(transformation_tree_.saccade_offset_vec * PUPIL_IRIS_DELTA);
        left_iris_.setTransformation(temp_transform);

        temp_transform = transformation_tree_.right_eye_transform_;
        temp_transform.translate(transformation_tree_.gaze_offset_vec * PUPIL_IRIS_DELTA).translate(transformation_tree_.saccade_offset_vec * PUPIL_IRIS_DELTA);
        right_iris_.setTransformation(temp_transform);

        temp_transform = transformation_tree_.left_eye_transform_;
        temp_transform.translate(transformation_tree_.gaze_offset_vec).translate(transformation_tree_.saccade_offset_vec);
        left_pupil_.setTransformation(temp_transform);

        temp_transform = transformation_tree_.right_eye_transform_;
        temp_transform.translate(transformation_tree_.gaze_offset_vec).translate(transformation_tree_.saccade_offset_vec);
        right_pupil_.setTransformation(temp_transform);
    }

    void setExpression();

    void setPreset();

    void configure(const FaceConfiguration &face_config);

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
    {
        if (do_saccades_)
        {
            if (saccade_countdown_ <= 0)
            {

                std::cout << "do saccade" << std::endl;

                // const sf::Vector2f delta = sf::Vector2f(gaze_vector.x * MAX_GAZE_SIZE.x, gaze_vector.y * MAX_GAZE_SIZE.y);
                transformation_tree_.saccade_offset_vec = sf::Vector2f(saccade_pos_dist_(random_engine_), saccade_pos_dist_(random_engine_));

                sf::Transform temp_transform = transformation_tree_.left_eye_transform_;
                temp_transform.translate(transformation_tree_.gaze_offset_vec * PUPIL_IRIS_DELTA).translate(transformation_tree_.saccade_offset_vec * PUPIL_IRIS_DELTA);
                left_iris_.setTransformation(temp_transform);

                temp_transform = transformation_tree_.right_eye_transform_;
                temp_transform.translate(transformation_tree_.gaze_offset_vec * PUPIL_IRIS_DELTA).translate(transformation_tree_.saccade_offset_vec * PUPIL_IRIS_DELTA);
                right_iris_.setTransformation(temp_transform);

                temp_transform = transformation_tree_.left_eye_transform_;
                temp_transform.translate(transformation_tree_.gaze_offset_vec).translate(transformation_tree_.saccade_offset_vec);
                left_pupil_.setTransformation(temp_transform);

                temp_transform = transformation_tree_.right_eye_transform_;
                temp_transform.translate(transformation_tree_.gaze_offset_vec).translate(transformation_tree_.saccade_offset_vec);
                right_pupil_.setTransformation(temp_transform);

                saccade_countdown_ = saccade_time_dist_(random_engine_);
            }
            else
            {
                saccade_countdown_ -= frame_delta_time;
            }
        }

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

        transformation_tree_.draw(renderWindow, frame_delta_time);
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
    TransformationTree transformation_tree_;

    bool do_saccades_;
    // counts down milliseconds until next saccade
    float saccade_countdown_;
    std::normal_distribution<float> saccade_time_dist_;
    std::mt19937 random_engine_;
    // position to move for a saccade
    std::normal_distribution<float> saccade_pos_dist_;

    bool DEBUG_ONCE;
    float DEBUG_TIMER;
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

    transformation_tree_.left_eye_transform_ = sf::Transform::Identity;
    transformation_tree_.left_eye_transform_.combine(face_center_transform);
    transformation_tree_.left_eye_transform_.translate(-0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);
    transformation_tree_.left_eye_transform_.scale(face_config.pupil_scaling);

    transformation_tree_.right_eye_transform_ = sf::Transform::Identity;
    transformation_tree_.right_eye_transform_.combine(face_center_transform);
    transformation_tree_.right_eye_transform_.translate(0.5f * face_config.eye_spacing * WINDOW_WIDTH, WINDOW_HEIGHT * face_config.eye_y);
    transformation_tree_.right_eye_transform_.scale(face_config.pupil_scaling);

    sf::Transform left_eyebrow_eye_transform;
    left_eyebrow_eye_transform.combine(transformation_tree_.left_eye_transform_);
    left_eyebrow_eye_transform.translate(0, face_config.eyebrow_spacing * WINDOW_HEIGHT);
    left_eyebrow_eye_transform.scale(face_config.eyebrow_scaling);

    sf::Transform right_eyebrow_eye_transform;
    right_eyebrow_eye_transform.combine(transformation_tree_.right_eye_transform_);
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
    do_saccades_ = face_config.should_do_saccades;

    // iris
    left_iris_.setShape(face_config.iris_shape);
    left_iris_.setTransformation(transformation_tree_.left_eye_transform_);
    left_iris_.setColour(face_config.iris_colour);
    left_iris_.setSquircleRadius(face_config.iris_squircle_radius);
    left_iris_.show(face_config.show_iris);

    right_iris_.setShape(face_config.iris_shape);
    right_iris_.setTransformation(transformation_tree_.right_eye_transform_);
    right_iris_.setColour(face_config.iris_colour);
    right_iris_.setSquircleRadius(face_config.iris_squircle_radius);
    right_iris_.show(face_config.show_iris);

    // left pupil
    left_pupil_.setTransformation(transformation_tree_.left_eye_transform_);
    left_pupil_.setColour(face_config.pupil_colour);
    left_pupil_.setSquircleRadius(face_config.pupil_squircle_radius);
    left_pupil_.setPupilHighlightShow(face_config.show_pupil_highlights);

    // right pupil
    right_pupil_.setTransformation(transformation_tree_.right_eye_transform_);
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
