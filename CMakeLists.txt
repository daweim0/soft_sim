cmake_minimum_required(VERSION 2.8.4)
project(mytest)

set(EXTRA_FLAGS -g3 -O0)

add_executable(explore
    odesim/explore.cpp
    odesim/Mesh.cpp
)
target_compile_options(explore PUBLIC ${EXTRA_FLAGS} -Wall -std=c++11 -I/usr/include/eigen3)
target_link_libraries(explore PRIVATE -L/usr/local/lib -lode -ldrawstuff -lGL -lGLU -lX11 -pthread -lm -ltinyxml)

add_executable(sim
    odesim/sim.cpp
    odesim/Mesh.cpp
)
target_compile_options(sim PUBLIC ${EXTRA_FLAGS} -Wall -std=c++11 -I/usr/include/eigen3)
target_link_libraries(sim PRIVATE -L/usr/local/lib -lode -ldrawstuff -lGL -lGLU -lX11 -pthread -lm -ltinyxml)
