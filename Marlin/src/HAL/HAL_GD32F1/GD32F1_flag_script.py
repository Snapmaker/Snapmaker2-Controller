# extra script for linker and compile options
Import("env")
env.Append(
  LINKFLAGS = [ "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard" ], # -mfpu is required here since GCC is outdated
  CCFLAGS=[ "-mfpu=fpv4-sp-d16",
            "-mfloat-abi=hard",

            "-fsigned-char",
            "-fno-move-loop-invariants",
            "-fno-strict-aliasing",

            "--specs=nano.specs",
            "--specs=nosys.specs",

            "-DTARGET_GD32F1"
  ],
)
