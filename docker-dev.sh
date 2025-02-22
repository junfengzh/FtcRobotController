# start docker dev container
docker run -it -v .:/home/mobiledevops/app mobiledevops/android-sdk-image bash

# inside the dev container
# - ./gradlew
# - adb connect <host:port>
# - adb install -r TeamCode/build/outputs/apk/release/TeamCode-release.apk
