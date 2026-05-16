# 主机 src 组件清单。
# PlatformIO + ESP-IDF 会从这里收集 src 下的所有源码并注册为应用组件。
# 这里不要写主从共享构建逻辑；主机和从机必须继续作为两个独立工程编译、烧录。
# REQUIRES 中显式列出 Arduino、esp_wifi 和 esp_timer，是因为当前 ESP-NOW、
# 串口入口和 200us / 5kHz 控制定时分别依赖它们。

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "../include"
    REQUIRES framework-arduinoespressif32 esp_wifi esp_timer
)
