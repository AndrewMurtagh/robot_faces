# robot_faces Robot Operating System package


[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/AndrewMurtagh/robot_faces/graphs/commit-activity)
[![GitHub license](https://img.shields.io/github/license/AndrewMurtagh/robot_faces.svg)](https://github.com/AndrewMurtagh/robot_faces/blob/master/LICENSE)
[![GitHub contributors](https://img.shields.io/github/contributors/AndrewMurtagh/robot_faces.svg)](https://GitHub.com/AndrewMurtagh/robot_faces/graphs/contributors/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](http://makeapullrequest.com)



## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [To Run](#to-run)
- [API](#api)
- [Ideas](#ideas)
- [TODO](#todo)
- [Contributors](#contributors)
- [License](#license)


## Introduction

robot_faces is a ROS 1 package written in C++11 for rendering animated robot faces. Almost everything is parametised and can be changed through dynamic_recofigure to produce a variety of faces. All configuration of the face support several emotional expressions and actions such as speaking and blinking.


## Dependencies

We use SFML to render animations. It does not come with a FindSFML.cmake so one was added in the `cmake/` directory.

This package is compatible with ROS 1. However, implementation of the face was hidden behind a facade and seperated from ROS as much as possible to ease porting to ROS 2.


## To Run

`rosrun robot_faces robot_faces_node` or `roslaunch robot_faces robot_face.launch`

Roslaunch is recommended.


## API

The face can be interacted with through ROS services.

### Expression

You can change the expression of the face to a number of predefined expressions listed below through the a `/robot_face/expression` service call.

1. NEUTRAL

2. SADNESS

3. FEAR

4. DISGUST

5. ANGER

6. JOY

7. HAPPINESS

8. AWE

9. SURPRISE

The default expression is `NEUTRAL`. You can set a timeout in milliseconds for how long the expression should be held for, after it expires the expression will return to its previous state. If the timeout is set to zero the change in expression will be permanent.

Below is an example of permanently changing the expression to surprise. The expression value is case-insensitive.

```
rosservice call /robot_face/expression "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
expression: 'surprise'
timeout: 0"
```

### Gaze

You can direct the gaze of the eyes through the `/robot_face/gaze` service call. Elevation is up and down and and Azimuth is left and right. Each variable ranges between -1.0 and 1.0 where -1.0 and 1.0 are the maximum extent of the gaze and 0.0 is looking straight ahead. Positive azimuth looks to the face's left, negative azimuth looks to the face's right, positive elevation looks up and negative elevation looks down.

As with the expression service call, you can set a timeout in milliseconds after which, the gaze will return to center; or leave it at zero to permanently change the gaze.

```
rosservice call /robot_face/gaze "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
elevation: 0.0
azimuth: 0.0
timeout: 0"
```

## Ideas

- Include face parameter presets.

- Nonlinear easing and interpolation.

- Different eyelid shapes.

- Wink, yawn, shed tears, use emojis.

- Expression intensities.


## TODO

- [ ] Gaze

- [ ] Expressions

- [ ] Saccades

- [ ] Handle window size changing and resize entities

- [ ] Make squircle mouth inherit from squircle entity

- [ ] Reset to defaults in dynamic_reconfigure

- [ ] Set full screen window

- [ ] Let annulus nose x and y vary independently

- [ ] Check for changes in face config before sending to face facade

- [ ] Seperate header and implementation files

## Contributors

- Andrew Murtagh (murtagan@tcd.ie)

- More contributors are welcome.

## License

Published under [MIT license](LICENSE).

