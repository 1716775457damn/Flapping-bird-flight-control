# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "E:/code/esp32/idf/blink/build/bootloader"
  "E:/code/esp32/idf/blink/build/bootloader-prefix"
  "E:/code/esp32/idf/blink/build/bootloader-prefix/tmp"
  "E:/code/esp32/idf/blink/build/bootloader-prefix/src/bootloader-stamp"
  "E:/code/esp32/idf/blink/build/bootloader-prefix/src"
  "E:/code/esp32/idf/blink/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/code/esp32/idf/blink/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/code/esp32/idf/blink/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
