cmake_minimum_required(VERSION 2.6)
project(myproject)
find_package(Qt4 REQUIRED)

include_directories(${QT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

set(myProject_SRCS myProject.cpp main.cpp)

qt4_automoc(${myProject_SRCS})
add_executable(myproject ${myProject_SRCS})
target_link_libraries(myproject ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

install(TARGETS myproject RUNTIME DESTINATION bin)
