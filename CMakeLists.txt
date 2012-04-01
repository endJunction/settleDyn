cmake_minimum_required(VERSION 2.8)

PROJECT(settleDyn CXX)

SET(WARNING_ERROR_FLAGS "-Wall -Wextra -pedantic -std=c++11")
SET(CMAKE_CXX_FLAGS "-pipe -march=native -frounding-math ${WARNING_ERROR_FLAGS}")

SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-ggdb -pg -O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0")

INCLUDE_DIRECTORIES(/usr/include/bullet)

LINK_LIBRARIES(
    BulletMultiThreaded
    BulletDynamics
    BulletCollision
    LinearMath 
    pthread
)

ADD_EXECUTABLE(settleDyn src/main.cpp)
