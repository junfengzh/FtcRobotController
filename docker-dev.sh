# start docker dev container
docker run -it -v .:/home/mobiledevops/app mobiledevops/android-sdk-image bash

# inside the dev container
# - ./gradlew clean assemblyRelease
# - adb connect <host:port>
# - adb install -r TeamCode/build/outputs/apk/release/TeamCode-release.apk
