# Swerve Drive

This module integrates with the Road Runner motion planning library. For general tuning concepts, please refer to the [road runner docs](https://rr.brott.dev/docs/v1-0/tuning/).

## Usage

Modify the `SwerveDrive` class to match your robot's physical characteristics. This includes things like wheel diameter, wheel positions, and gear ratios.

Two `WheelUnit` implementations are provided for different types of swerve modules ( `DifferentialWheel` and `ServoCoaxialWheel` ). Different `WheelUnit` can be used in the same robot, so you can mix and match as needed.

To be compatible with the Road Runner system, all distance units should be in inches, and all angle units should be in radians. 

An angle sensor should be installed on each wheel unit to provide feedback for the drive system. Each kind of `WheelUnit` class is designed to work with an angle sensor, and it will use the sensor's readings to control the wheel's orientation.

Modify the `AngleSensor` class to match the type of angle sensor you are using. 

## Tuning

Before using the `SVATuning` OpMode, you need to set a basic P (proportional) gain for the wheel angle PID controller to ensure the wheels can point in the correct direction. The initial value does not need to be precise.

`SVATuning` OpMode will help you to find the best feedforward parameters. It will rotate and translate your robot, testing `kS`, `kV`,  `kJ`(Effective moment of inertia) and `kM`(Equivalent Mass) parameters, and then calculate the best parameters for you.

`kA` in feedforward is not tested in `SVATuning` OpMode, because it is calculated by `kM` and `kJ`. If your testing ground is not big enough, you can ignore `kM`, set `kA`(can be set to `kJ` to get a acceptable behaviour) and use `vector` input instead of `translation_rotation` input (to `WheelUnit`s) in `SwerveController` to avoid testing `kM` parameter (which requires a long translational distance).

PID parameters are not tested in `SVATuning` OpMode, and can be set to 0 (like other drive implements, and they do not use PID). You can tune them by yourself after you get the feed forward parameters.

## Build

Suggested way is to download dependencies from maven. Let Gradle sync and download all required dependencies. Just use Android Studio to open this repo.

To build everything by yourself, see the [Downloader of Swerve](https://github.com/BluePowerRobotics/Swerve).

Then, follow the instructions in the README of the Downloader.

After that, you should:

- remove `//` in line 8~10,19 in build.gradle
- remove `//` in line 3~7     in settings.gradle
- remove `//` in line 34~38   in TeamCode/build.gradle
- add    `//` in line 39~42   in TeamCode/build.gradle
