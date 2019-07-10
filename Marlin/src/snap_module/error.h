#ifndef ERROR_H_
#define ERROR_H_


#define COMMON_ERR_BASE   0

#define E_SUCCESS         (COMMON_ERR_BASE + 0)     /* non error */

#define E_FAILURE         (COMMON_ERR_BASE + 1)     /* common error code */

#define E_PARAM           (COMMON_ERR_BASE + 2)     /* got a invalid parameter */

#define E_NO_MEM          (COMMON_ERR_BASE + 3)     /* apply memory failed */

#define E_NO_RESRC        (COMMON_ERR_BASE + 4)     /* apply resource failed except memory */

#define E_BUSY            (COMMON_ERR_BASE + 5)     /* resource is busy, for example, bus is busy,
                                                     * a mutex lock is busy
                                                     */
#define E_TIMEOUT         (COMMON_ERR_BASE + 6)

#define E_HARDWARE        (COMMON_ERR_BASE + 7)     /* hardware errors such as invalid bus state */

#endif // #ifndef ERROR_H_
