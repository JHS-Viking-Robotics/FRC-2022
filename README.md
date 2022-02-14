# FRC-2022

[![Run on Repl.it](https://repl.it/badge/github/jhs-viking-robotics/FRC-2022)](https://repl.it/github/jhs-viking-robotics/FRC-2022)

Team 7221 code for the 2022 FRC season. This project can be found on GitHub at [JHS-Viking-Robotics/FRC-2022](https://github.com/JHS-Viking-Robotics/FRC-2022). You can also find the 2021 code for Heimdall on the ```main-heimdall-2021``` branch.

New to this project? Head on over to our [Java-Hello-World tutorial](https://github.com/JHS-Viking-Robotics/Java-Hello-World) and return back here once you feel comfortable with Java. Then check out the [Getting Started](#getting-started) section for some quick tips, and happy hacking!

## Contents

- [Quick Setup](#quick-setup)
- [Getting Started](#getting-started)
  - [Hardware Setup](#hardware-setup)
  - [Deploying To RoboRIO](#deploying-to-roborio)
  - [Controls Layout](#controls-layout)
  - [Style Guide](#style-guide)
  - [Other Notes](#other-notes)
- [Feature Requests And Bug Reporting](#feature-requests-and-bug-reporting)
- [Learn More About The Team](#learn-more-about-the-team)

## Quick Setup

Using a computer set up as a Driver Station, pull this repository from GitHub using the instructions on the big green ```Code``` button. Connect the computer to the robot with a USB or Ethernet cable. In VS Code, press ```Ctrl + Shift + P``` to bring up the command palette, type ```WPILib: Deploy Robot Code```, and hit enter. Now you should be able to drive the robot using the Driver Station.

## Getting Started

The following sections contain useful information if you want to deploy or develop this code.

### Hardware Setup

Our robot is still being built, but follows the following conventions. All devices are updated with the latest firmware, and are always connected pos <--> pos and grd <--> grd (inversion is only handled in the code). You can find pictures of the robot [on our website](https://www.jhsvikingrobotics.com/this-year-s-robots).

Subsystem | Motor(s) | Controller(s) | Sensor(s)
:-:|:-:|:-:|:-:

### Deploying To RoboRIO

To deploy the code to the RoboRIO, pull this repository from GitHub using the instructions on the big green ```Code``` button. Choose the tag on the ```main``` branch depending on which version you need, or leave it alone to use the latest version. Code on the ```development``` branch is under testing and not necessarily functional.

### Controls Layout

There are very few button mappings to the controller right now, as everything is in the testing phase. To operate the robot, open the ```Driver Station``` and open the ```Shuffleboard```. There are tabs for each subsystem, which have controllers on them.

### Style Guide

We do not yet have a strict style guide. Generally we expect new commits to follow the same styling as the rest of the code.

All methods, members, and code blocks should have javadocs and comments to make them easily understandable to other developers.

For pull requests and commit messages, check out the [Team 7221 Docs](https://jhs-viking-robotics.github.io/Java-Hello-World/) for templates and guidelines. In general, always add a high level description of functionality or the interface.

### Other Notes

Xbox controller inputs are inverted for the y axis, which needs to be handled whenever passing joystick ax(es) to a command. See the following diagram:

```plaintext
    (-)
     |
(-)--+--(+)
     |
    (+)
```

New features can be added in a PR to the development branch after basic has been completed. After extensive testing, several new features will be merged together to main, and tagged with a new release.

## Feature Requests And Bug Reporting

Find something broken, or see something you don't like? Open a Bug Report or a Feature Request using the provided templates. We'll do our best to help out!

## Learn More About The Team

Interested in sponsoring us or getting to know our team? Check out [our website](https://www.jhsvikingrobotics.com/)!
