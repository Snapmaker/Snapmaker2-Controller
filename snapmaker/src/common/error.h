/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_ERROR_H_
#define SNAPMAKER_ERROR_H_

#include <stdio.h>

typedef uint8_t ErrCode;

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

#define E_INVALID_STATE   (COMMON_ERR_BASE + 8)     /* state is invalid for current operation */

#define E_SAME_STATE      (COMMON_ERR_BASE + 9)     /* current state is same with new state */

#define E_INVALID_CMD     (COMMON_ERR_BASE + 10)     /* invalid event id or operation code */


#define DATA_LINK_ERR_BASE (COMMON_ERR_BASE + 190)

#define E_NO_SOF                (DATA_LINK_ERR_BASE + 0)     /* didn't found SOF in RX buffer */
#define E_NO_HEADER             (DATA_LINK_ERR_BASE + 1)     /* cannot get header in RX buffer */
#define E_INVALID_DATA_LENGTH   (DATA_LINK_ERR_BASE + 2)     /* incorrect checksum for data length or it is out of range */
#define E_NO_DATA               (DATA_LINK_ERR_BASE + 3)     /* cannot get data field in RX buffer */
#define E_INVALID_DATA          (DATA_LINK_ERR_BASE + 2)     /* incorrect checksum of data */

#define PRIVATE_ERROR_BASE  200
#define E_NO_SWITCHING_STA  (PRIVATE_ERROR_BASE + 0)
#define E_NO_WORKING        (PRIVATE_ERROR_BASE + 1)
#define E_NO_FILAMENT       (PRIVATE_ERROR_BASE + 2)
#define E_DOOR_OPENED       (PRIVATE_ERROR_BASE + 3)
#define E_REJECT_SYNC_WRITE (PRIVATE_ERROR_BASE + 4)
#define E_AUTO_PROBING      (PRIVATE_ERROR_BASE + 5)
#define E_IS_HOMING         (PRIVATE_ERROR_BASE + 6)
#define E_NO_HOMED          (PRIVATE_ERROR_BASE + 7)

#endif // #ifndef ERROR_H_
