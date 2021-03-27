#ifndef FACE_CONFIG_H
#define FACE_CONFIG_H

#include <SFML/Graphics.hpp>
#include <robot_faces/consts.hpp>

/*
why not just pass the dynamic reconfigure object to face and let it
deal with that? 
we want to have separate any face code from any ros code as much as possible

this also provides a method to only updates parts of the face that need updating
recomputing vertices can be expensive so we don't do it if we don't have to
*/
struct FaceConfiguration
{

    // no need for defaults since dynamiic reconfigure will call it
    // immediately
    // method fo isDifferent();
    // update if different with map

    /*
    misc
    */
    float window_width;
    float window_height;
    sf::Color background_colour;
    float face_center;

    /*
    Eyebrows
    */
    EyebrowShape eyebrow_shape;
    float eyebrow_spacing;
    sf::Vector2f eyebrow_scaling;
    sf::Color eyebrow_colour;
    bool show_eyebrows;

    /*
    Eyes
    */
    int avg_blink_interval;
    bool should_blink;
    bool should_do_saccades;
    float eye_spacing;
    float eye_y;

    bool show_pupil_highlights;
    sf::Vector2f pupil_squircle_radius;
    sf::Vector2f pupil_scaling;
    sf::Color pupil_colour;
    bool show_pupil;
    
    IrisShape iris_shape;
    sf::Vector2f iris_squircle_radius;
    sf::Vector2f iris_scaling;
    sf::Color iris_colour;
    bool show_iris;

    /*
    Nose
    */
    NoseShape nose_shape;
    float nose_y;
    sf::Vector2f nose_scaling;
    sf::Vector2f nose_squircle_radius;
    sf::Color nose_colour;
    bool show_nose;


    /*
    Mouth
    */
    MouthShape mouth_shape;
    int mouth_radius;
    float mouth_y;
    sf::Vector2f mouth_scaling;
    sf::Vector2f mouth_squircle_radius;
    sf::Color mouth_colour;
    bool show_mouth;


};

#endif // FACE_CONFIG_H