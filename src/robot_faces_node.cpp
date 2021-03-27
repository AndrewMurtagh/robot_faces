
#include <fstream>
#include <string>
#include <iostream>
#include <random>
#include <regex>
#include <map>

#include <SFML/Graphics.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <robot_faces/ParametersConfig.h>

#include <robot_faces/face.hpp>
#include <robot_faces/face-config.hpp>

/*
global variables
*/
sf::RenderWindow renderWindow(sf::VideoMode(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT), "robot_face");
sf::Clock frame_clock; // gets time from one frame to the next
Face face;
FaceConfiguration faceConfig;

/*
callback functions
*/

void dynamicReconfigureCb(robot_faces::ParametersConfig &config, uint32_t level)
{
    ROS_INFO("\nReconfigure Request");

    /*
    misc
    */
    renderWindow.setSize(sf::Vector2u(config.window_width, config.window_height));
    faceConfig.window_width = config.window_width;
    faceConfig.window_height = config.window_height;
    try
    {
        g_background_colour = validateColour(config.background_colour);
        faceConfig.background_colour = g_background_colour;
    }
    catch (...)
    {
        ROS_ERROR("colour is formatted incorrectly.");
    }
    faceConfig.face_center = config.face_center;

    /*
    Eyebrows
    */
    faceConfig.eyebrow_shape = static_cast<EyebrowShape>(config.eyebrow_shape);
    faceConfig.eyebrow_spacing = config.eyebrow_spacing;
    faceConfig.eyebrow_scaling = sf::Vector2f(config.eyebrow_scaling_x, config.eyebrow_scaling_y);
    try
    {
        sf::Color colour = validateColour(config.eyebrow_colour);
        faceConfig.eyebrow_colour = colour;
    }
    catch (...)
    {
        ROS_ERROR("colour is formatted incorrectly.");
    }
    faceConfig.show_eyebrows = config.show_eyebrows;

    /*
    Eyes
    */
    faceConfig.avg_blink_interval = config.avg_blink_interval;
    faceConfig.should_blink = config.should_blink;
    faceConfig.should_do_saccades = config.should_do_saccades;
    faceConfig.eye_spacing = config.eye_spacing;
    faceConfig.eye_y = config.eye_y;
    faceConfig.show_pupil_highlights = config.show_pupil_highlights;
    faceConfig.pupil_squircle_radius = sf::Vector2f(config.pupil_squircle_radius_x, config.pupil_squircle_radius_y);
    faceConfig.pupil_scaling = sf::Vector2f(config.pupil_scaling_x, config.pupil_scaling_y);
    faceConfig.iris_squircle_radius = sf::Vector2f(config.iris_squircle_radius_x, config.iris_squircle_radius_y);
    faceConfig.iris_scaling = sf::Vector2f(config.iris_scaling_x, config.iris_scaling_y);
    faceConfig.iris_shape = static_cast<IrisShape>(config.iris_shape);
    try
    {
        sf::Color iris_colour = validateColour(config.iris_colour);
        faceConfig.iris_colour = iris_colour;

        sf::Color pupil_colour = validateColour(config.pupil_colour);
        faceConfig.pupil_colour = pupil_colour;
    }
    catch (...)
    {
        ROS_ERROR("colour is formatted incorrectly.");
    }
    faceConfig.show_iris = config.show_iris;
    faceConfig.show_pupil = config.show_pupil;

    /*
    Nose
    */
    faceConfig.nose_shape = static_cast<NoseShape>(config.nose_shape);
    faceConfig.nose_y = config.nose_y;
    faceConfig.nose_scaling = sf::Vector2f(config.nose_scaling_x, config.nose_scaling_y);
    faceConfig.nose_squircle_radius = sf::Vector2f(config.nose_squircle_radius_x, config.nose_squircle_radius_y);
    try
    {
        sf::Color nose_colour = validateColour(config.nose_colour);
        faceConfig.nose_colour = nose_colour;
    }
    catch (...)
    {
        ROS_ERROR("colour is formatted incorrectly.");
    }
    faceConfig.show_nose = config.show_nose;

    /*
    Mouth
    */
    faceConfig.mouth_shape = static_cast<MouthShape>(config.mouth_shape);
    faceConfig.mouth_y = config.mouth_y;
    faceConfig.mouth_scaling = sf::Vector2f(config.mouth_scaling_x, config.mouth_scaling_y);
    faceConfig.mouth_squircle_radius = sf::Vector2f(config.mouth_squircle_radius_x, config.mouth_squircle_radius_y);
    try
    {
        sf::Color mouth_colour = validateColour(config.mouth_colour);
        faceConfig.mouth_colour = mouth_colour;
    }
    catch (...)
    {
        ROS_ERROR("colour is formatted incorrectly.");
    }
    faceConfig.show_mouth = config.show_mouth;

    /*
    configure
    */
    face.configure(faceConfig);
}

/*
main
*/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_face");

    ros::NodeHandle node_handle;

    dynamic_reconfigure::Server<robot_faces::ParametersConfig> server;
    dynamic_reconfigure::Server<robot_faces::ParametersConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigureCb, _1, _2);
    server.setCallback(f);

    // set the framerate to be the same as the monitor's refresh rate to reduce the change of adverse visual artifacts - tearing.
    // sometimes vertical synchronistion is forced off by the graphic card so we should fall back to limiting the framerate
    // to a reasonable figure.
    // renderWindow.setVerticalSyncEnabled(true);
    renderWindow.setFramerateLimit(30);

    /*
    main loop
    */

    while (ros::ok() && renderWindow.isOpen())
    {

        /*
        check for window close event
        */
        sf::Event event;
        while (renderWindow.pollEvent(event))
        {

            if (event.type == sf::Event::Closed)
            {
                renderWindow.close();
            }
        }

        /* 
        frame timing
        */
        float frame_delta_time = frame_clock.getElapsedTime().asMilliseconds() / 1000.0f; // time since last frame draw
        frame_clock.restart();

        /*
        draw
        */
        renderWindow.clear(g_background_colour);
        face.draw(renderWindow, frame_delta_time);
        renderWindow.display();
        // std::cout << std::endl;

        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
