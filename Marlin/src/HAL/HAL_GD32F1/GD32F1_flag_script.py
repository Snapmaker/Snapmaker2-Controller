# extra script for linker and compile options
Import("env")
env.Append(
  CCFLAGS=[ "-Os",
            "-mcpu=cortex-m4",
#           "-mfpu=fpv4-sp-d16",
#           "-mfloat-abi=hard",
            "-mthumb",

            "-fsigned-char",
            "-fno-move-loop-invariants",
            "-fno-strict-aliasing",

            "--specs=nano.specs",
            "--specs=nosys.specs",

            "-IMarlin/src/HAL/GD32F1",

            "-MMD",
            "-MP",
            "-DTARGET_GD32F1"
  ],
  CFLAGS=["-std=gnu11"],
)
