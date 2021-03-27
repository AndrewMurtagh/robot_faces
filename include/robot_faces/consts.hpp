#ifndef CONSTS_H
#define CONSTS_H

#include <regex>

enum class EyebrowShape
{
    Rectangle,
    Square,
    Rounded,
    Straight,
    Arch,
    Arc
};

enum class IrisShape
{
    Thick,
    Oval,
    Almond,
    Arc,
    Squircle
};

enum class NoseShape
{
    Annulus,
    Curve,
    Animal,
    Squircle
};

enum class MouthShape
{
    Squircle,
    Line
};


std::regex g_rgba_regex("^(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9]),?(25[0-5]|2[0-4][0-9]|1[0-9]?[0-9]?|[1-9][0-9]?|[0-9])$");
sf::Color g_background_colour(255, 255, 255, 255);
const sf::Transform g_mirror_transform(-1.0f, 0.f, 0.0f,
                                        0.f, 1.0f, 0.0f,
                                        0.f, 0.f, 1.f);

const float CLOSE_ENOUGH_THRESHOLD          = 3.0f; // in px
const float MOVEMENT_SPEED                  = 150; // in px / second
const float PUPIL_IRIS_DELTA                = 0.7f; // the difference in movement between the pupil and iris
const int NUM_CORNER_POINTS                 = 25; // the number of points used to discretize a corner

// window
const int DEFAULT_WINDOW_WIDTH              = 800;
const int DEFAULT_WINDOW_HEIGHT             = 600;

const int DEFAULT_MOUTH_WIDTH               = 200;
const int DEFAULT_MOUTH_HEIGHT              = 60;
const int DEFAULT_RECT_MOUTH_RADIUS         = 30;

const sf::Vector2f IRIS_SIZE                = sf::Vector2f(120.0f, 120.0f);
const sf::Vector2f SQUIRCLE_IRIS_RADIUS     = sf::Vector2f(1.0f, 1.0f);

const int PUPIL_SIZE                        = 80;
const sf::Vector2f PUPIL_RADIUS             = sf::Vector2f(1.0f, 1.0f);

// nose
const float ANNULUS_NOSE_THICKNESS          = 0.3f; // 30% of the size
const sf::Vector2f NOSE_SIZE                = sf::Vector2f(60.0f, 60.0f);
const sf::Vector2f SQUIRCLE_NOSE_RADIUS     = sf::Vector2f(1.0f, 1.0f);
const float CURVE_NOSE_THICKNESS            = 5.0f;
const int CURVE_NOSE_RADIUS                 = 30;

// mouth 
const float LINE_MOUTH_THICKNESS            = 16.0f; //px

// reference markers
const sf::Color REF_MARKER_COLOUR           = sf::Color(sf::Color::Cyan);
const int REF_MARKER_RADIUS                 = 5; // pixels

#endif // CONSTS_H