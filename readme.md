# WARP Skystone

This repo contains FTC code for the 2020-2021 Wasatch Academy Robotics Program.

## Side-loading driver app via adb on Mac

1. First enable USB debugging on phone. After enabling developer options, enable USB debugging.
1. Add Android Studio platform-tools to the path.

   ```sh
   echo 'export ANDROID_HOME=/Users/$USER/Library/Android/sdk' >> ~/.bash_profile
   echo 'export PATH=${PATH}:$ANDROID_HOME/tools:$ANDROID_HOME/platform-tools' >> ~/.bash_profile
   source ~/.bash_profile
   ```

1. Confirm that it worked.

   ```sh
   adb devices
   ```

1. Install teamcode onto the phone.

   ```sh
   adb install PATH_TO_FTC_REPO/doc/apk/FtcDriverStation-release.apk
   ```

## Make symbolic link to teamcode directory

```sh
mkdir ~/symlinks
ln -s ~/PATH_TO_FTC_REPO/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ ~/symlinks/warp
export CDPATH=~/symlinks
cd warp
```

## How to push code onto the control hub over wifi

1. Connect over wifi to control hub direct wifi network.

1. Connect via adb.

   ```sh
   adb connect 192.168.43.1:5555
   ```

1. Check connection or disconnect.

   ```sh
   adb devices
   adb disconnect
   ```

## How to connect to internet and control with secondary USB wifi adapter on Mac

1. Get drivers for wifi adapter

1. Open network preferences, press the gears icon, then set the service order as necessary.