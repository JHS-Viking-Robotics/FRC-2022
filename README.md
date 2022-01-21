# FRC-2022

Team 7221 code for the 2022 FRC season. This project can be found on GitHub at [JHS-Viking-Robotics/FRC-2022](https://github.com/JHS-Viking-Robotics/FRC-2022)

During the off-season, this code will be developed for the 2020 season bot Heimdall (see the 2020 competition code [here](https://github.com/JHS-Viking-Robotics/FRC-2020)). After Kickoff Day on January 8th, we will start development for the 2022 FRC Competition. You can find the competition info on the [FIRST Robotics Website](https://www.firstinspires.org/).

New to this project? Head on over to our [Java-Hello-World tutorial](https://github.com/JHS-Viking-Robotics/Java-Hello-World) and return back here once you feel comfortable with Java. Then check out the [Getting Started](#getting-started) section for some quick tips, and happy hacking!

## Quick Setup

Using a computer set up as a [Driver Station](https://jhs-viking-robotics.github.io/Java-Hello-World/Install-Software/Windows-10), pull this repository from GitHub using the instructions on the big green ```Code``` button. Connect the computer to the robot with a USB or Ethernet cable. In ```WPILib 2021```, press ```Ctrl + Shift + P``` to bring up the command palette, type ```WPILib: Deploy Robot Code```, and hit enter. Now you should be able to open the ```Driver Station``` program and connect to the robot. See [Deploying to RoboRIO](#deploying-to-roborio) if you have issues.

## Contents

- [Getting Started](#getting-started)
  - [Hardware Setup](#hardware-setup)
  - [Deploying To RoboRIO](#deploying-to-roborio)
  - [Controls Layout](#controls-layout)
  - [Style Guide](#style-guide)
  - [Other Notes](#other-notes)
- [Feature Requests And Bug Reporting](#feature-requests-and-bug-reporting)
- [Learn More About The Team](#learn-more-about-the-team)

## Getting Started

The following sections contain useful information if you want to deploy or develop this code.

### Hardware Setup

Our robot, Heimdall, contains the following general hardware. All devices are updated with the latest firmware, and are always connected pos <--> pos and grd <--> grd (inversion is only handled in the code). You can find pictures of the robot [on our website](https://www.jhsvikingrobotics.com/this-year-s-robots).

Subsystem | Motor(s) | Controller(s) | Sensor(s)
:-:|:-:|:-:|:-:
Drivetrain (primary motors) | [AndyMark CIM Motor](https://www.andymark.com/categories/motors) | [CTRE Talon SRX](http://www.ctr-electronics.com/control-system/motor-control/talon-srx.html) | [CTRE Magnetic Encoder](http://www.ctr-electronics.com/sensors/srx-magnetic-encoder.html)
Drivetrain (follower motors) | [AndyMark CIM Motor](https://www.andymark.com/categories/motors) | [CTRE Victor SPX](http://www.ctr-electronics.com/control-system/motor-control/victor-spx.html) | N/A
Hopper Lift | [VexPro Bag Motor](https://www.vexrobotics.com/217-3351.html) | [CTRE Talon SRX](http://www.ctr-electronics.com/control-system/motor-control/talon-srx.html) | [CTRE Magnetic Encoder](http://www.ctr-electronics.com/sensors/srx-magnetic-encoder.html)
Hopper Intake | [AndyMark 775](https://www.andymark.com/categories/motors) | [CTRE Victor SPX](http://www.ctr-electronics.com/control-system/motor-control/victor-spx.html) | N/A

### Deploying To RoboRIO

This code can be deployed from any [Windows/macOS/Linux computer]((https://jhs-viking-robotics.github.io/Java-Hello-World/Install-Software/Windows-10)).

To deploy the code to the RoboRIO, pull this repository from GitHub using the instructions on the big green ```Code``` button. Choose the tag on the ```main``` branch depending on which version you need, or leave it alone to use the latest version. Code on the ```development``` branch is under testing and not necessarily functional. In ```WPILib 2021```, press ```Ctrl + Shift + P``` to bring up the command palette, type ```WPILib: Deploy Robot Code```, and hit enter. There should be a message indicating if the deploy was successful.

To connect to and drive the robot, a Windows computer set up as a [Driver Station](https://jhs-viking-robotics.github.io/Java-Hello-World/Install-Software/Windows-10) needs to be connected to the robot over WiFi, ethernet, or usb. Open the ```Driver Station``` program, make sure the team number is entered, and that ```Shuffleboard``` is selected as the dashboard.

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
