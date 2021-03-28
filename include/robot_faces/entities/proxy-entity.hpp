#ifndef PROXY_ENTITY_H
#define PROXY_ENTITY_H

#include <SFML/Graphics.hpp>
#include <robot_faces/entities/ientity.hpp>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>
#include <robot_faces/utils.hpp>

template <typename T>
class ProxyEntity : public IEntity
{
protected:
    // defined above public so it is defined in constructor
    typedef typename std::multimap<T, std::shared_ptr<Entity>> EntityMap;
    typedef typename std::multimap<T, std::shared_ptr<Entity>>::iterator EntityMapItr;
    typedef typename std::pair<T, std::shared_ptr<Entity>> EntityMapPair;
    T shape_;
    bool show_;
    EntityMap entity_map_;
    sf::Transform curr_transformation_;
    sf::Transform target_transformation_;

public:
    ProxyEntity(T shape, EntityMap entity_map) : shape_(shape),
                                                 entity_map_(entity_map),
                                                 show_(true)
    {
        curr_transformation_ = sf::Transform(1.0f, 0.0f, 800 * 0.5f,
                                            0.0f, 1.0f, 600 * 0.75f,
                                            0.0f, 0.0f, 1.f);

        target_transformation_ = sf::Transform(1.0f, 0.0f, 800 * 0.5f,
                                            0.0f, 1.0f, 600 * 0.75f,
                                            0.0f, 0.0f, 1.f);
    }

    void setShape(T shape)
    {
        shape_ = shape;
    }

    void moveTo()
    {
    }
    void show(const bool show)
    {
        show_ = show;
    }

    /*
    IEntity
    */
    void setTransformation(const sf::Transform transform) override
    {
        target_transformation_ = transform;
        // curr_transformation_ = target_transformation_;
        // for (EntityMapPair entity : entity_map_)
        // {
        //     // TODO DON'T MOVE IMMEDIATELY, INTERLOP
        //     entity.second->setTransformation(transform);
        // }
    }

    void setColour(const sf::Color colour) override
    {
        for (EntityMapPair entity : entity_map_)
        {
            entity.second->setColour(colour);
        }
    }

    void setExpression(const int TEMP_EXPRESSION) override
    {
        for (EntityMapPair entity : entity_map_)
        {
            entity.second->setExpression(TEMP_EXPRESSION);
        }
    }

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override
    {
        if (!show_)
            return;

        const float* curr_trans_matrix = curr_transformation_.getMatrix();
        const float* target_trans_matrix = target_transformation_.getMatrix();

        sf::Transform new_curr_transformation(
            curr_trans_matrix[0] + frame_delta_time * 0.01f * (target_trans_matrix[0] - curr_trans_matrix[0]),
            curr_trans_matrix[4] + frame_delta_time * 0.01f * (target_trans_matrix[4] - curr_trans_matrix[1]),
            curr_trans_matrix[12] + frame_delta_time * 0.01f * (target_trans_matrix[12] - curr_trans_matrix[12]),

            curr_trans_matrix[1] + frame_delta_time * 0.01f * (target_trans_matrix[1] - curr_trans_matrix[1]),
            curr_trans_matrix[5] + frame_delta_time * 0.01f * (target_trans_matrix[5] - curr_trans_matrix[5]),
            curr_trans_matrix[13] + frame_delta_time * 0.01f * (target_trans_matrix[13] - curr_trans_matrix[13]), 

            curr_trans_matrix[3] + frame_delta_time * 0.01f * (target_trans_matrix[3] - curr_trans_matrix[3]),
            curr_trans_matrix[7] + frame_delta_time * 0.01f * (target_trans_matrix[7] - curr_trans_matrix[7]),
            curr_trans_matrix[15] + frame_delta_time * 0.01f * (target_trans_matrix[15] - curr_trans_matrix[15]));
        

        curr_transformation_ = new_curr_transformation;

        for (EntityMapPair entity : entity_map_)
        {
            entity.second->setTransformation(curr_transformation_);

            if (entity.first == shape_)
            {
                entity.second->draw(renderWindow, frame_delta_time);
            }
        }

    }
};

#endif // PROXY_ENTITY_H