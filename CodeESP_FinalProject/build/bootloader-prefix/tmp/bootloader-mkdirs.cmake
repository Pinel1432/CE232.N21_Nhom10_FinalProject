# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v4.4.4/components/bootloader/subproject"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix/tmp"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix/src/bootloader-stamp"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix/src"
  "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/WorkSpace/Shool_Practice/ESP32/Test_LAB4_WiFi/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
