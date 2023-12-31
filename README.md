# Polygon-visibility

## Overview
The Polygon Visibility project is a C++ code that allows users to interactively explore and visualize the visibility of a randomly generated polygon from different points. This program provides a graphical user interface where users can control the display of visibility polygons and the non-empty core of the polygon.

## Requirements
- Visual Studio 2019
- CMake 3.15.4
- [VVR Framework](https://github.com/vvrgroup/VVR-Framework)
  
## Usage
1. Press the mouse at different spots to visualize the visibility polygons.
2. Press the "2" key to display the non-empty core of the polygon. The non-empty core is the space within the polygon where the user can see the entire polygon.
3. Press the "3" key to generate a new random polygon.

## Results
![Demo](./images/1.png)
- The first image shows a randomly generated polygon.
- From a random viewpoint, the second image displays the polygon as seen from that specific spot.
- The third image illustrates the non-empty core of the polygon along with the visible polygon as seen from a viewpoint within the non-empty core.

## License

The framework used in this project is licensed under the terms of the [MIT License](./framework_license/LICENSE.txt).


This project borrows and utilizes the [VVR Framework](https://github.com/vvrgroup/VVR-Framework) developed by VVR Group. The VVR Framework is a C++ API of useful utilities, including mathematics, graphics, and other functionalities. It provides a set of powerful tools that assist in various aspects of development.

For more information about the framework and its original source, please visit the [VVR Framework repository](https://github.com/vvrgroup/VVR-Framework).
