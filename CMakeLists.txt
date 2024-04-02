cmake_minimum_required(VERSION 3.20)

project(SyNSlicerEngine LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
qt_standard_project_setup()

find_package(VTK REQUIRED)

find_package(CGAL)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(spdlog)

set(CLIPPER2_INC
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.core.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.engine.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.export.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.minkowski.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.offset.h
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include/clipper2/clipper.rectclip.h
)

set(CLIPPER2_SRC
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/src/clipper.engine.cpp
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/src/clipper.offset.cpp
    $ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/src/clipper.rectclip.cpp
)

file(GLOB_RECURSE SyNSlicerEngine
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/*.h"
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/*.cpp"
)

file(GLOB_RECURSE SyNSlicerEngineObject
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/Object/*.h"
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/Object/*.cpp"
)

file(GLOB_RECURSE SyNSlicerEngineAlgorithm
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/Algorithm/*.h"
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/Algorithm/*.cpp"
)

file(GLOB_RECURSE SyNSlicerGUI
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/*.h"
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/*.cpp"
)

file(GLOB_RECURSE SyNSlicerGUIPathPlanning
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/PathPlanning/*.h"
    "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/PathPlanning/*.cpp"
)


add_library(Clipper2 ${CLIPPER2_INC} ${CLIPPER2_SRC})
target_include_directories(Clipper2 PUBLIC "$ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include")

source_group("Object"    FILES ${SyNSlicerEngineObject})
source_group("Algorithm" FILES ${SyNSlicerEngineAlgorithm})

add_library(SyNSlicerEngine ${SyNSlicerEngine} ${SyNSlicerEngineObject} ${SyNSlicerEngineAlgorithm})
target_include_directories(SyNSlicerEngine PUBLIC "$ENV{libigl_DIR}/include")
target_include_directories(SyNSlicerEngine PUBLIC "$ENV{CLIPPER2_DIR}/CPP/Clipper2Lib/include")
target_include_directories(SyNSlicerEngine PUBLIC "$ENV{GTE_PATH}/include/GTE")
target_include_directories(SyNSlicerEngine PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/obejct")
target_include_directories(SyNSlicerEngine PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/algorithm")
target_include_directories(SyNSlicerEngine PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine")

target_link_libraries(SyNSlicerEngine PUBLIC CGAL::CGAL)
target_link_libraries(SyNSlicerEngine PUBLIC Eigen3::Eigen)
target_link_libraries(SyNSlicerEngine PUBLIC Clipper2)
target_link_libraries(SyNSlicerEngine PUBLIC spdlog::spdlog)

target_precompile_headers(SyNSlicerEngine PUBLIC ${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/pch_SyNSlicerEngine.h)

source_group("PlanPlanning" FILES ${SyNSlicerGUIPathPlanning})

add_executable(SyNSlicerGUI ${SyNSlicerGUI} ${SyNSlicerGUIPathPlanning})
target_include_directories(SyNSlicerGUI PUBLIC "$ENV{libigl_DIR}/include")
target_include_directories(SyNSlicerGUI PUBLIC "$ENV{GTE_PATH}/include/GTE")
target_include_directories(SyNSlicerGUI PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI")
target_include_directories(SyNSlicerGUI PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/PathPlanning")
target_include_directories(SyNSlicerGUI PUBLIC "${CMAKE_SOURCE_DIR}/src/SyNSlicerEngine/obejct")

target_link_libraries(SyNSlicerGUI PUBLIC Qt6::Widgets)
target_link_libraries(SyNSlicerGUI PUBLIC ${VTK_LIBRARIES})
target_link_libraries(SyNSlicerGUI PUBLIC CGAL::CGAL)
target_link_libraries(SyNSlicerGUI PUBLIC Eigen3::Eigen)
target_link_libraries(SyNSlicerGUI PUBLIC SyNSlicerEngine)
target_link_libraries(SyNSlicerGUI PUBLIC spdlog::spdlog)

target_precompile_headers(SyNSlicerGUI PUBLIC ${CMAKE_SOURCE_DIR}/src/SyNSlicerGUI/pch_SyNSlicerGUI.h)
