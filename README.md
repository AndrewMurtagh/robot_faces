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

The face can be interacted with through ROS serivces.

### Speaking

You can set the face to speaking or not through the `/robot_face/speaking` service. Example:

```
rosservice call /robot_face/speaking "speak: true"
```

### Expression

You can change the expression of the face to a number of predefined expressions listed below through the `/robot_face/expression` service. The default expression is `NEUTRAL`.  The supported expressions are:

1. `NEUTRAL`

2. `SAD`

3. `SCARED`

4. `ANGRY`

5. `HAPPY`

6. `SHOCKED`


The type of the `expression` in the service is `string` and is case-insensitive. Values not on the above list will be ignored. Example of a call to the expression service.

```
rosservice call /robot_face/expression "expression: 'HAPPY'"
```


### Gaze

You can direct the gaze of the eyes through the `/robot_face/gaze` service call. Elevation is up and down and and Azimuth is left and right. Each variable ranges between -1.0 and 1.0 where -1.0 and 1.0 are the maximum extent of the gaze and 0.0 is looking straight ahead. Positive azimuth looks to the face's left, negative azimuth looks to the face's right, positive elevation looks up and negative elevation looks down. Any value outside of -1.0 and 1.0 inclusive will be clamped. It is up to the client to determine the value of gaze based on their particular application. 

```
rosservice call /robot_face/gaze "elevation: -0.7
azimuth: 1.0"
```

## Ideas

- Include face parameter presets.

- Nonlinear easing and interpolation.

- Different eyelid shapes.

- Wink, yawn, shed tears, use emojis.

- Expression intensities.

## TODO

- [ ] Expressions

- [ ] Handle window size changing and resize entities

- [ ] Reset to defaults option in dynamic_reconfigure

- [ ] Set full screen window

- [ ] Let annulus nose x and y vary independently

- [ ] Check for changes in face config before sending to face facade

- [ ] Seperate header and implementation files

- [ ] Add timeout and speed to speaking service.

- [ ] Add timeout to gaze service.

- [x] Saccades

- [x] Gaze

- [x] Mouth speaking

- [x] Make squircle mouth inherit from squircle entity

## Notes

- Should we make a Mouth entity that deals with speaking behaviour instead of duplicating code in LineMouth and SquircleMouth?

- Should speaking randomisation have a minimum distance to prevent small movements? Should we also enforce a change in direction with every change?

## Contributors

- Andrew Murtagh (murtagan@tcd.ie)

- More contributors are welcome.

## License

Published under [MIT license](LICENSE).

