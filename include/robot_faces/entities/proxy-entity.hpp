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
    ProxyEntity(T, EntityMap);

    void setShape(T);

    void show(const bool);

    /*
    IEntity
    */
    void setTransformation(const sf::Transform transform) override;

    void setColour(const sf::Color colour) override;

    void setExpression(const Expression expression) override;

    void draw(sf::RenderWindow &renderWindow, const float frame_delta_time) override;
};

template<typename T>
ProxyEntity<T>::ProxyEntity(T shape, EntityMap entity_map) : shape_(shape),
                                             entity_map_(entity_map),
                                             show_(true)
{
}

template<typename T>
void ProxyEntity<T>::setShape(T shape)
{
    shape_ = shape;
}

template<typename T>
void ProxyEntity<T>::show(const bool show)
{
    show_ = show;
}

/*
IEntity
*/
template<typename T>
void ProxyEntity<T>::setTransformation(const sf::Transform transform)
{
    target_transformation_ = transform;
}

template<typename T>
void ProxyEntity<T>::setColour(const sf::Color colour)
{
    for (EntityMapPair entity : entity_map_)
    {
        entity.second->setColour(colour);
    }
}

template<typename T>
void ProxyEntity<T>::setExpression(const Expression expression)
{
    for (EntityMapPair entity : entity_map_)
    {
        entity.second->setExpression(expression);
    }
}

template<typename T>
void ProxyEntity<T>::draw(sf::RenderWindow &renderWindow, const float frame_delta_time)
{
    if (!show_)
        return;

    const float *curr_trans_matrix = curr_transformation_.getMatrix();
    const float *target_trans_matrix = target_transformation_.getMatrix();

    sf::Transform new_curr_transformation(
        curr_trans_matrix[0] + frame_delta_time * SPEED * (target_trans_matrix[0] - curr_trans_matrix[0]),  
        curr_trans_matrix[4] + frame_delta_time * SPEED * (target_trans_matrix[4] - curr_trans_matrix[1]),
        curr_trans_matrix[12] + frame_delta_time * SPEED * (target_trans_matrix[12] - curr_trans_matrix[12]),

        curr_trans_matrix[1] + frame_delta_time * SPEED * (target_trans_matrix[1] - curr_trans_matrix[1]),
        curr_trans_matrix[5] + frame_delta_time * SPEED * (target_trans_matrix[5] - curr_trans_matrix[5]),
        curr_trans_matrix[13] + frame_delta_time * SPEED * (target_trans_matrix[13] - curr_trans_matrix[13]),

        curr_trans_matrix[3] + frame_delta_time * SPEED * (target_trans_matrix[3] - curr_trans_matrix[3]),
        curr_trans_matrix[7] + frame_delta_time * SPEED * (target_trans_matrix[7] - curr_trans_matrix[7]),
        curr_trans_matrix[15] + frame_delta_time * SPEED * (target_trans_matrix[15] - curr_trans_matrix[15]));

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

#endif // PROXY_ENTITY_H