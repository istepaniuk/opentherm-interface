cmake_minimum_required(VERSION 3.7)
set(PROJECT_NAME Blink)
project(${PROJECT_NAME} CXX)

set(CMAKE_CXX_STANDARD 11)

###########################################################################
# Settings - might need to be adjusted for different boards

# Path variables
set(arduinoInstallDir ~/opt/arduino-1.8.12)
set(arduinoPackagesDir ~/.arduino15/packages)
set(arduinoLibrariesDir ~/Arduino/libraries)

# Variant
set(variant mega)
# Library version (check in Arduino IDE: Tools -> Board -> Boards Manager)
set(libraryVersion 1.6.21)

###########################################################################

# Headers
include_directories(${arduinoInstallDir}/hardware/arduino/avr/cores/arduino/
        ${arduinoInstallDir}/hardware/tools/avr/avr/include/)

# Variant specific configuration (pins_arduino.h)
include_directories(${arduinoInstallDir}/hardware/arduino/avr/variants/${variant}/)

###########################################################################

# Libraries
# 1) In Arduino install directory
FILE(GLOB Libs ${arduinoInstallDir}/libraries/*/src/)
include_directories(${Libs})

# 2) In the sketchbook directory (User-installed libraries)
FILE(GLOB LibDirs ${arduinoLibrariesDir}/*)
include_directories(${LibDirs})

# 3) In the hardware specific directory (such as SPI or EEPROM)
FILE(GLOB Packages ${arduinoPackagesDir}/arduino/hardware/avr/${libraryVersion}/libraries/*/src/)
include_directories(${Packages})

###########################################################################
# Some hacks for better syntax highlighting

# Include Arduino.h in every source file
add_definitions(-include Arduino.h)

# For Serial (HardwareSerial.hpp)
add_definitions(-DUBRRH)

# The WString.h file is not correctly handled by CLion and therefore the String class produces a lot of syntax errors.
# Copying WString.h, renaming the #ifndef String_class_h, and including the modified header magically resolves this problem.
execute_process(COMMAND bash -c "mktemp" OUTPUT_VARIABLE stringDummyHeaderFile)
string(STRIP ${stringDummyHeaderFile} stringDummyHeaderFile)

execute_process(COMMAND bash -c "cp ${arduinoInstallDir}/hardware/arduino/avr/cores/arduino/WString.h ${stringDummyHeaderFile}")
execute_process(COMMAND bash -c "sed -i -e 's/String_class_h/String_dummy_class_h/g' ${stringDummyHeaderFile}")

add_definitions(-include ${stringDummyHeaderFile})
set(sourceFiles ${stringDummyHeaderFile})

###########################################################################

# Tell CMake which files to include
file(GLOB_RECURSE sourceFiles ${sourceFiles} *.cpp *.hpp *.ino)
add_executable(${PROJECT_NAME} ${sourceFiles})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
