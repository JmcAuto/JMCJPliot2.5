# Docker image build process

## Pre download

```bash
scp -r caros@10.89.114.243:/mnt/drivers/ /jmcauto/docker/build/
scp -r caros@10.89.114.243:/mnt/yolo_camera_detector /jmcauto/docker/build/
```

## Usage

Simply run
```bash
./build_dev.sh ./dev.x86_64.dockerfile
./build_dev.sh ./yolo.dockerfile
```

## Add new installer

The best practice of a new installer would be:

1. Well tested.

   Of course. Make it work, and don't break other installers, such as
   incompatible versions of libraries.

1. Standalone.

   Have minimum assumption about the basement, which means, you can depend on
   the base image and installers/pre_install.sh. Other than that, you should
   install all the dependencies in your own installer.

1. Thin.

   It will generate a new layer in the final image, so please keep it as thin as
   possible. For example, clean up all intermediate files:

   ```bash
   wget xxx.zip
   # Unzip, make, make install
   rm -fr xxx.zip xxx
   ```

1. Cross-architecture.

   It would be awesome to work perfectly for different architectures such as X86
   and ARM.
