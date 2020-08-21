#ifndef SNAPMAKER_CONFIG_H_
#define SNAPMAKER_CONFIG_H_

#define XSTR_(M) #M
#define XSTR(M) XSTR_(M)

#define SNAPMAKER_SRC(path)   XSTR(../../snapmaker/src/path)

#define MARLIN_SRC(path)  XSTR(../../../Marlin/src/path)

#include MARLIN_SRC(inc/MarlinConfig.h)

#define MARLIN_HAL(header)  XSTR(../../../Marlin/src/HAL/HAL_PLATFORM/header)


#endif  // #ifndef SNAPMAKER_CONFIG_H_
