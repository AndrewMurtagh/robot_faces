#ifndef PROXY_ENTITY_H
#define PROXY_ENTITY_H

#include <SFML/Graphics.hpp>
#include <robot_faces/entities/ientity.hpp>
#include <robot_faces/entities/entity.hpp>
#include <robot_faces/consts.hpp>

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

public:
    ProxyEntity(T shape, EntityMap entity_map) : shape_(shape),
                                                 entity_map_(entity_map),
                                                 show_(true) {}

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
        for (EntityMapPair entity : entity_map_)
        {
            entity.second->setTransformation(transform);
        }
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

        for (std::pair<EntityMapItr, EntityMapItr> range(entity_map_.equal_range(shape_)); range.first != range.second; ++range.first)
        {
            range.first->second->draw(renderWindow, frame_delta_time);
        }
    }
};

#endif // PROXY_ENTITY_H