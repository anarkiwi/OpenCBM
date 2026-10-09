/*
 *  xum1541 plugin interface, derived from the xu1541 file of the same name
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 *  Copyright 2009-2010 Nate Lawson <nate@root.org>
 *  Copyright 1999-2001 Michael Klein <michael(dot)klein(at)puffin(dot)lb(dot)shuttle(dot)de>
 *  Copyright 2001-2005, 2007, 2010 Spiro Trikaliotis
 */

/*! **************************************************************
** \file lib/plugin/xum1541/s1_s2_pp.c \n
** \author Till Harbaum, Spiro Trikaliotis \n
** \n
** \brief Shared library / DLL for accessing the driver: Code for accessing fast protocols of xum1541
**
****************************************************************/

#ifdef WIN32
#include <windows.h>
#include <windowsx.h>

/*! Mark: We are in user-space (for debug.h) */
#define DBG_USERMODE

/*! Mark: We are building the DLL */
// #define DBG_DLL

/*! The name of the executable */
#define DBG_PROGNAME "OPENCBM-XUM1541.DLL"

/*! This file is "like" debug.c, that is, define some variables */
// #define DBG_IS_DEBUG_C

#include "debug.h"
#endif

#include <stdlib.h>

//! mark: We are building the DLL */
#define OPENCBM_PLUGIN
//#include "i_opencbm.h"
#include "archlib.h"

#include "xum1541.h"


/*
 * Read with a trailing status block (firmware version 9 and later).
 * Returns the count only if all bytes were transferred, else -1.
 */
static int
xum1541_read_status(CBM_FILE HandleDevice, unsigned char mode, unsigned char *data, unsigned int size)
{
    int status, count;

    if (xum1541_read_ext((struct opencbm_usb_handle *)HandleDevice, mode,
        data, size, &status, &count) < 0 || count != (int)size || status != count)
        return -1;
    return count;
}

/*
 * Write with a trailing status block (firmware version 9 and later).
 * Returns the count only if all bytes were transferred, else -1.
 */
static int
xum1541_write_status(CBM_FILE HandleDevice, unsigned char mode, const unsigned char *data, unsigned int size)
{
    int status, count;

    if (xum1541_write_ext((struct opencbm_usb_handle *)HandleDevice, mode,
        data, size, &status, &count) < 0 || count != (int)size || status != count)
        return -1;
    return count;
}

/*-------------------------------------------------------------------*/
/*--------- OPENCBM ARCH FUNCTIONS ----------------------------------*/

/*! \brief Read data with serial1 protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_s1_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion >= 9)
        return xum1541_read_status(HandleDevice, XUM1541_S1 | XUM_RW_STATUS, data, size);
    return xum1541_read((struct opencbm_usb_handle *)HandleDevice, XUM1541_S1, data, size);
}

/*! \brief Write data with serial1 protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_s1_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion >= 9)
        return xum1541_write_status(HandleDevice, XUM1541_S1 | XUM_RW_STATUS, data, size);
    return xum1541_write((struct opencbm_usb_handle *)HandleDevice, XUM1541_S1, data, size);
}

/*! \brief Read data with serial2 protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_s2_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion >= 9)
        return xum1541_read_status(HandleDevice, XUM1541_S2 | XUM_RW_STATUS, data, size);
    return xum1541_read((struct opencbm_usb_handle *)HandleDevice, XUM1541_S2, data, size);
}

/*! \brief Write data with serial2 protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes actually written, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_s2_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion >= 9)
        return xum1541_write_status(HandleDevice, XUM1541_S2 | XUM_RW_STATUS, data, size);
    return xum1541_write((struct opencbm_usb_handle *)HandleDevice, XUM1541_S2, data, size);
}

/*! \brief Read data with parallel protocol (d64copy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_pp_dc_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    return xum1541_read((struct opencbm_usb_handle *)HandleDevice, XUM1541_PP, data, size);
}

/*! \brief Write data with parallel protocol (d64copy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes actually written, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_pp_dc_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    return xum1541_write((struct opencbm_usb_handle *)HandleDevice, XUM1541_PP, data, size);
}

/*! \brief Read data with parallel protocol (cbmcopy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_pp_cc_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    return xum1541_read((struct opencbm_usb_handle *)HandleDevice, XUM1541_P2, data, size);
}

/*! \brief Write data with parallel protocol (cbmcopy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes actually written, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_pp_cc_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    return xum1541_write((struct opencbm_usb_handle *)HandleDevice, XUM1541_P2, data, size);
}

/*! \brief Read data with burst nibbler protocol (cbmcopy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes actually read, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_nib_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    return xum1541_read((struct opencbm_usb_handle *)HandleDevice, XUM1541_NIB, data, size);
}

/*! \brief Write data with burst nibbler protocol (cbmcopy)

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes actually written, 0 on device error. If there is a
    fatal error, returns -1.
*/
int CBMAPIDECL
opencbm_plugin_nib_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    return xum1541_write((struct opencbm_usb_handle *)HandleDevice, XUM1541_NIB, data, size);
}

/*! \brief Read data with X protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes read, or -1 if not all of them could be read or
    the firmware does not support the protocol.
*/
int CBMAPIDECL
opencbm_plugin_x_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 9)
        return -1;
    return xum1541_read_status(HandleDevice, XUM1541_X, data, size);
}

/*! \brief Write data with X protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes written, or -1 if not all of them could be written
    or the firmware does not support the protocol.
*/
int CBMAPIDECL
opencbm_plugin_x_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 9)
        return -1;
    return xum1541_write_status(HandleDevice, XUM1541_X, data, size);
}

/*! \brief Read data with X protocol at 2 MHz drive speed

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes read, or -1 if not all of them could be read or
    the firmware does not support the protocol.
*/
int CBMAPIDECL
opencbm_plugin_x2_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 9)
        return -1;
    return xum1541_read_status(HandleDevice, XUM1541_X | XUM_X_2MHZ, data, size);
}

/*! \brief Write data with X protocol at 2 MHz drive speed

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes written, or -1 if not all of them could be written
    or the firmware does not support the protocol.
*/
int CBMAPIDECL
opencbm_plugin_x2_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 9)
        return -1;
    return xum1541_write_status(HandleDevice, XUM1541_X | XUM_X_2MHZ, data, size);
}

/*! \brief Set the xum1541 firmware I/O idle timeout

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param ms
    Timeout in milliseconds, rounded up to 100 ms units. 0 disables it.

  \return
    0 on success, -1 on error.
*/
int CBMAPIDECL
opencbm_plugin_xum1541_set_timeout(CBM_FILE HandleDevice, unsigned int ms)
{
    return xum1541_set_timeout((struct opencbm_usb_handle *)HandleDevice, ms);
}

/*
 * Burst X transfer (firmware version 10 and later), mode XUM1541_X with
 * XUM_X_BURST and optionally XUM_X_2MHZ. Returns the count only if all bytes
 * were transferred, else -1, also when the firmware is older.
 */
static int
xum1541_xb_read(CBM_FILE HandleDevice, unsigned char flags, unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 10)
        return -1;
    return xum1541_read_status(HandleDevice, XUM1541_X | XUM_X_BURST | flags, data, size);
}

static int
xum1541_xb_write(CBM_FILE HandleDevice, unsigned char flags, const unsigned char *data, unsigned int size)
{
    if (DeviceFirmwareVersion < 10)
        return -1;
    return xum1541_write_status(HandleDevice, XUM1541_X | XUM_X_BURST | flags, data, size);
}

/*! \brief Read data with burst X protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes read, or -1 if not all of them could be read or
    the firmware is older than version 10.
*/
int CBMAPIDECL
opencbm_plugin_xb_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    return xum1541_xb_read(HandleDevice, 0, data, size);
}

/*! \brief Write data with burst X protocol

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes written, or -1 if not all of them could be written
    or the firmware is older than version 10.
*/
int CBMAPIDECL
opencbm_plugin_xb_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    return xum1541_xb_write(HandleDevice, 0, data, size);
}

/*! \brief Read data with burst X protocol at 2 MHz drive speed

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer which will hold the read bytes.

  \param size
    The size of the data buffer the read bytes will be written to.

  \return
    The number of bytes read, or -1 if not all of them could be read or
    the firmware is older than version 10.
*/
int CBMAPIDECL
opencbm_plugin_xb2_read_n(CBM_FILE HandleDevice, unsigned char *data, unsigned int size)
{
    return xum1541_xb_read(HandleDevice, XUM_X_2MHZ, data, size);
}

/*! \brief Write data with burst X protocol at 2 MHz drive speed

  \param HandleDevice
    A CBM_FILE which contains the file handle of the driver.

  \param data
    Pointer to the data buffer to be written

  \param size
    The size of the data buffer to be written

  \return
    The number of bytes written, or -1 if not all of them could be written
    or the firmware is older than version 10.
*/
int CBMAPIDECL
opencbm_plugin_xb2_write_n(CBM_FILE HandleDevice, const unsigned char *data, unsigned int size)
{
    return xum1541_xb_write(HandleDevice, XUM_X_2MHZ, data, size);
}
