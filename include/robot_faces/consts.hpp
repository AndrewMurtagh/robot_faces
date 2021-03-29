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
const sf::Transform g_mirror_transform(-1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f);

const int FRAME_RATE                        = 30; // fps
const int NUM_CORNER_POINTS                 = 25; // the number of points used to discretize a corner

// window
const int DEFAULT_WINDOW_WIDTH              = 800; // px
const int DEFAULT_WINDOW_HEIGHT             = 600; // px

// eyes
const float MEAN_SACCADE_TIME               = 240.0f; // ms
const float STDDEV_SACCADE_TIME             = 50.0f; // ms
const float STDDEV_SACCADE_POS              = 8.0f; // px
const float EYE_SPEED                       = 0.01f; // px / ms
const sf::Vector2f MAX_GAZE_SIZE            = sf::Vector2f(120.0f, 80.0f);
const float PUPIL_IRIS_DELTA                = 0.8f; // the difference in movement between the pupil and iris
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
const sf::Vector2f MOUTH_SIZE               = sf::Vector2f(200.0f, 100.0f);
const sf::Vector2f SQUIRCLE_MOUTH_RADIUS    = sf::Vector2f(1.0f, 1.0f);
const float SPEAKING_ELON_CLOSE_ENOUGH      = 0.01f; // Y scale
const float SPEAKING_SPEED                  = 0.01f; // px / ms

// reference markers
const sf::Color REF_MARKER_COLOUR           = sf::Color(sf::Color::Cyan);
const int REF_MARKER_RADIUS                 = 5; // pixels

#endif // CONSTS_H