include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)

add_library(canlib
    ../src/canbus.cpp
    ../src/sensornode.cpp
    ../src/controllernode.cpp
)

target_include_directories(canlib PUBLIC ../include)

add_executable(run_tests
    test_cansimulator.cpp
)

target_link_libraries(run_tests
    canlib
    GTest::gtest_main
)

target_include_directories(run_tests PRIVATE ../include)

include(GoogleTest)
gtest_discover_tests(run_tests)
