cmake_minimum_required(VERSION 3.22)
project(Project1HostTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(ecg_hr_tests
        ecg_hr_test.c
        ../Hardware/Src/ecg_hr.c
        ../Hardware/Src/ecg_filter.c
)

target_include_directories(ecg_hr_tests PRIVATE
        ../Hardware/Inc
)

target_compile_options(ecg_hr_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
)

target_link_libraries(ecg_hr_tests PRIVATE m)

add_test(NAME ecg_hr_tests COMMAND ecg_hr_tests)

add_executable(max30102_spo2_tests
        max30102_spo2_test.c
        ../Hardware/Src/max30102_spo2.c
)

target_include_directories(max30102_spo2_tests PRIVATE
        ../Hardware/Inc
)

target_compile_options(max30102_spo2_tests PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
)

target_link_libraries(max30102_spo2_tests PRIVATE m)

add_test(NAME max30102_spo2_tests COMMAND max30102_spo2_tests)
