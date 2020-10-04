from __future__ import print_function
import sys

#dynamic build flags for generic compile options
if __name__ == "__main__":
  args = " ".join([ 
                    "-Isnapmaker/lib/GD32F1/libraries/FreeRTOS1030",
                    "-Isnapmaker/lib/GD32F1/libraries/FreeRTOS1030/utility/include",
                    "-Isnapmaker/src",
                  ])


  for i in range(1, len(sys.argv)):
    args += " " + sys.argv[i]

  print(args)

