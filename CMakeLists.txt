cmake_minimum_required(VERSION 3.16)
project(HarmoSpray)

message(STATUS "Target System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "System Version: ${CMAKE_SYSTEM_VERSION}")
message(STATUS "System Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")


if(MSVC)
    # 移除默认的运行时检查选项
    string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/RTCsu" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/RTCs" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/RTCu" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

    # 对 C 标志也做同样处理
    string(REPLACE "/RTC1" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
    string(REPLACE "/RTCsu" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
    string(REPLACE "/RTCs" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
    string(REPLACE "/RTCu" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT JMCServerExecutor)

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PROJECT_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_OUTPUT_DIR}/debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_DIR}/release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_OUTPUT_DIR}/debug)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_DIR}/release)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${PROJECT_OUTPUT_DIR}/debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_DIR}/release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径
set(CMAKE_INSTALL_PREFIX "${PROJECT_ROOT_DIR}/install" CACHE PATH "Installation Directory" FORCE)

if(MSVC)
    add_compile_options(/utf-8)
endif()

# 三方dll拷贝
# 复制指定库的 DLL 到执行程序目录（仅 Windows）
# 参数：
# 1) 库的 bin 根目录（包含 debug 和 release 子目录）
# 2) 库名
function(copyDllToExecutorDir BIN_DIR LIB_NAME_DEBUG LIB_NAME_RELEASE)
    if(WIN32)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${BIN_DIR}/debug/${LIB_NAME_DEBUG}
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/${LIB_NAME_DEBUG})
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${BIN_DIR}/release/${LIB_NAME_RELEASE}
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/${LIB_NAME_RELEASE})
    endif()
endfunction()


set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_PREFIX_PATH ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
file(GLOB PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

#qt5_add_resources(PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.qrc)

add_executable(${PROJECT_NAME}
	${PROJECT_SOURCES}
)

# Qt
target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport)

# Eigen
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/eigen-3.4.0)

# sigslot
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/sigslot/include)

# occ
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/occ/include)
target_link_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/occ/lib/x64/${CMAKE_BUILD_TYPE})
file(GLOB OCC_LIB_NAME ${PROJECT_ROOT_DIR}/thirdParty/occ/lib/x64/${CMAKE_BUILD_TYPE}/*.lib)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OCC_LIB_NAME})

# JMCClient
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCClient/include)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_link_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCClient/lib/debug)
	target_link_libraries(${PROJECT_NAME} PRIVATE JMCClientd)
else()
	target_link_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCClient/lib/release)
	target_link_libraries(${PROJECT_NAME} PRIVATE JMCClient)
endif()

# JMCServer
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCServer/include)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_link_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCServer/lib/debug)
	target_link_libraries(${PROJECT_NAME} PRIVATE JMCServerd)
else()
	target_link_directories(${PROJECT_NAME} PRIVATE ${PROJECT_ROOT_DIR}/thirdParty/JMCServer/lib/release)
	target_link_libraries(${PROJECT_NAME} PRIVATE JMCServer)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
	-D_USE_MATH_DEFINES
	-DNOMINMAX)


# ============================================
# Install 配置
# ============================================
install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/JMCClient/bin/debug/
    ${PROJECT_ROOT_DIR}/thirdParty/JMCServer/bin/debug/
    ${PROJECT_ROOT_DIR}/thirdParty/occ/bin/x64/debug/
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/bin/*.dll
    DESTINATION ${PROJECT_OUTPUT_DIR}/debug
    CONFIGURATIONS Debug
    OPTIONAL
)

install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/mkspecs/
    DESTINATION ${PROJECT_OUTPUT_DIR}/debug/mkspecs
    CONFIGURATIONS Debug
    OPTIONAL
)

install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/plugins/
    DESTINATION ${PROJECT_OUTPUT_DIR}/debug/plugins
    CONFIGURATIONS Debug
    OPTIONAL
)

install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/JMCClient/bin/release/
    ${PROJECT_ROOT_DIR}/thirdParty/JMCServer/bin/release/
    ${PROJECT_ROOT_DIR}/thirdParty/occ/bin/x64/release/
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/bin/*.dll
    DESTINATION ${PROJECT_OUTPUT_DIR}/release
    CONFIGURATIONS Release
    OPTIONAL
)

install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/mkspecs/
    DESTINATION ${PROJECT_OUTPUT_DIR}/release/mkspecs
    CONFIGURATIONS Release
    OPTIONAL
)

install(DIRECTORY
    ${PROJECT_ROOT_DIR}/thirdParty/Qt5.15.2/plugins/
    DESTINATION ${PROJECT_OUTPUT_DIR}/release/plugins
    CONFIGURATIONS Release
    OPTIONAL
)