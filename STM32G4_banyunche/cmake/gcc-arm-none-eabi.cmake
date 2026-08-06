set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

# Only set compiler/tools if not already provided via -D or environment
# If C compiler is set with a full path, derive other tools from the same directory
if(NOT CMAKE_C_COMPILER)
    set(CMAKE_C_COMPILER            ${TOOLCHAIN_PREFIX}gcc)
endif()

# Determine toolchain binary directory and extension (once)
if(CMAKE_C_COMPILER MATCHES "/")
    get_filename_component(_TOOLCHAIN_DIR ${CMAKE_C_COMPILER} DIRECTORY)
    get_filename_component(_C_COMPILER_EXT ${CMAKE_C_COMPILER} LAST_EXT)
    set(_USE_FULL_PATH TRUE)
else()
    set(_USE_FULL_PATH FALSE)
endif()

if(NOT CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER          ${CMAKE_C_COMPILER})
endif()

if(_USE_FULL_PATH)
    if(NOT CMAKE_CXX_COMPILER)
        set(CMAKE_CXX_COMPILER      ${_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}g++${_C_COMPILER_EXT})
    endif()
    set(CMAKE_LINKER                ${_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}g++${_C_COMPILER_EXT})
    set(CMAKE_OBJCOPY               ${_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}objcopy${_C_COMPILER_EXT})
    set(CMAKE_SIZE                  ${_TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}size${_C_COMPILER_EXT})
else()
    if(NOT CMAKE_CXX_COMPILER)
        set(CMAKE_CXX_COMPILER      ${TOOLCHAIN_PREFIX}g++)
    endif()
    set(CMAKE_LINKER                ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_OBJCOPY               ${TOOLCHAIN_PREFIX}objcopy)
    set(CMAKE_SIZE                  ${TOOLCHAIN_PREFIX}size)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G491XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
