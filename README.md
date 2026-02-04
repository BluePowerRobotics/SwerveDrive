# Swerve Drive

Check out the [road runner docs](https://rr.brott.dev/docs/v1-0/tuning/).

## Build

Suggested way is to download dependences from maven. Gradle will do everything. Just use Android Studio to open this repo.

To build everything by yourself, see the [Downloader of Swerve](https://github.com/BluePowerRobotics/Swerve).

Then, follow the instructions in the README of the Downloader.

After that, you should:

- remove `//` in line 8~10,19 in build.gradle
- remove `//` in line 3~7     in settings.gradle
- remove `//` in line 34~38   in TeamCode/build.gradle
- add    `//` in line 39~42   in TeamCode/build.gradle
