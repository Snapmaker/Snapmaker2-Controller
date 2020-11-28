# extra script for linker and compile options
Import("env")
env.Append(
  CCFLAGS=[ "-mfpu=fpv4-sp-d16",
            "-mfloat-abi=softfp",

            "-fsigned-char",
            "-fno-move-loop-invariants",
            "-fno-strict-aliasing",

            "--specs=nano.specs",
            "--specs=nosys.specs",

            "-DTARGET_GD32F1"
  ],
)
