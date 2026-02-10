cmake_minimum_required(VERSION 3.20)
project(sonic_screwdriver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

set(ARDUINO_PATH "/usr/share/arduino")

add_compile_definitions(
	__AVR__
	__AVR_ATtiny85__
	ARDUINO=10819
	F_CPU=8000000L
)

add_library(arduino_dummy STATIC dummy.cpp sonic-screwdriver.ino)

target_include_directories(arduino_dummy PRIVATE
	/usr/share/arduino/avr/include
	/usr/share/arduino/ATTinyCore-1.5.2-include
	${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino
	${ARDUINO_PATH}/hardware/arduino/avr/variants/tiny
)

set_target_properties(arduino_dummy PROPERTIES
	LINKER_LANGUAGE CXX
)