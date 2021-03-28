#ifndef SQUIRCLE_MOUTH_HPP
#define SQUIRCLE_MOUTH_HPP

#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/entities/squircle-entity.hpp>
#include <robot_faces/consts.hpp>

class SquircleMouth : public SquircleEntity
{
public:
    SquircleMouth() : is_speaking_(false),
                      curr_elongation_scale_(1.0),
                      target_elongation_scale_(1.0),
                      elongation_dist_(0.2, 1.4),
                      SquircleEntity(MOUTH_SIZE, SQUIRCLE_MOUTH_RADIUS)
    {
        std::random_device rd;
        random_engine_.seed(rd());

        target_elongation_scale_ = elongation_dist_(random_engine_);
    }

    void setSpeaking(const bool speaking)
    {
        is_speaking_ = speaking;
        if (speaking)
        {
            target_elongation_scale_ = elongation_dist_(random_engine_);
        }
        else
        {
            target_elongation_scale_ = MOUTH_SIZE.y;
        }
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        if (is_speaking_)
        {
            if (abs(target_elongation_scale_ - curr_elongation_scale_) < SPEAKING_ELON_CLOSE_ENOUGH)
            {
                curr_elongation_scale_ = target_elongation_scale_;
                target_elongation_scale_ = elongation_dist_(random_engine_);
            }
            else
            {
                float delta = frame_delta_time * SPEAKING_SPEED * (target_elongation_scale_ - curr_elongation_scale_);
                curr_elongation_scale_ += delta;
            }
            squircle_shape_.setScale(1.0f, curr_elongation_scale_);
        }
        renderWindow.draw(squircle_shape_, transform_);
        Entity::draw(renderWindow, frame_delta_time);
    }

private:
    bool is_speaking_;

    float curr_elongation_scale_;
    float target_elongation_scale_;

    std::mt19937 random_engine_;
    std::uniform_real_distribution<float> elongation_dist_;
};
#endif // SQUIRCLE_MOUTH_HPP