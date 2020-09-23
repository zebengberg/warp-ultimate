# WARP Skystone

This repo contains FTC code for the 2020-2021 Wasatch Academy Robotics Program.


## Side-loading driver app via adb on Mac

1. First enable USB debugging on phone. After enabling developer options, enable USB debugging.
1. Add Android Studio platform-tools to the path.
```shell script
$ echo 'export ANDROID_HOME=/Users/$USER/Library/Android/sdk' >> ~/.bash_profile
$ echo 'export PATH=${PATH}:$ANDROID_HOME/tools:$ANDROID_HOME/platform-tools' >> ~/.bash_profile
$ source ~/.bash_profile
```
1. Confirm that it worked.
```shell script
$ adb devices
```
1. Install teamcode onto the phone.
```shell script
$ adb install PATH_TO_FTC_REPO/doc/apk/FtcDriverStation-release.apk

## Make symbolic link to teamcode directory

```shell script
mkdir ~/symlinks
ln -s ~/PATH_TO_FTC_REPO/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ ~/symlinks/warp
export CDPATH=~/symlinks
cd warp
```

## How to push code onto the control hub over wifi

See: https://blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots/