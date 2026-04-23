cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_attendance_esp32_ble)

# Build-time provisioning for the attendance device.
# Usage:
#   idf.py build -DPROJECT_ID=my_project_doc_id
# If not provided, firmware will return "unknown" to the frontend.
if(NOT DEFINED PROJECT_ID OR "${PROJECT_ID}" STREQUAL "")
  set(PROJECT_ID "unknown")
endif()

# Define as a C string literal.
add_compile_definitions(PROJECT_ID="${PROJECT_ID}")

