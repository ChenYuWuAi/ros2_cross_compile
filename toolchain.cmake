set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定交叉编译的编译器
set(TRIPLE ${CMAKE_SYSTEM_PROCESSOR}-linux-gnu)

set(CMAKE_C_COMPILER /usr/bin/${TRIPLE}-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/${TRIPLE}-g++)
set(CMAKE_LINKER /usr/bin/${TRIPLE}-ld)

# Generic
function(require_env name)
if("$ENV{${name}}" STREQUAL "")
    message(FATAL_ERROR "Required environment variable ${name} not defined")
endif()
endfunction()

require_env(SYSROOT)
require_env(ROS_WS_INSTALL_PATH)
require_env(ROS_DISTRO)

# 设置 CMake 根路径为目标平台的库和头文件路
set(CMAKE_SYSROOT $ENV{SYSROOT})
set(OPENSSL_ROOT_DIR $ENV{SYSROOT}/usr/lib/aarch64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH $ENV{ROS_WS_INSTALL_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(PYTHON_SOABI cpython-${PY_VERSION}-${TRIPLE})
set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)