/**
********************************************************************************
\file       lcd-c5socarm.c

\brief      LCD functions for Altera Cyclone-V HPS LCD

This implementation uses the Altera Cyclone-V HPS LCD available on
Altera Cyclone-V development board(D) for handling LCD module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2016, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2015, Kalycito Infotech Private Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <alt_cache.h>
#include <alt_interrupt.h>
#include <alt_i2c.h>
#include <alt_globaltmr.h>
#include <alt_timers.h>
#include <socal/hps.h>
#include <socal/socal.h>

#include <system.h>
#include <sleep.h>
#include <trace/trace.h>
#include "lcdl.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//TODO Replace the I2C address from system.h
#define LCD_I2C_ADDRESS         (0x50 >> 1) // I2C address of LCD module
#define LCD_I2C_SPEED           40000       // I2C bus speed for accessing LCD module
#define LCD_ESCAPE_CHAR         0xfe        // Escape character used to prefix commands
#define LCD_POS_1ST_LINE        0x00        // LCD cursor position for 1st line
#define LCD_POS_2ND_LINE        0x40        // LCD cursor position for 2nd line
#define LCD_PRINT_DELAY_US      500         // Delay in us after printing text on LCD
#define LCDL_COLUMN             16          // Maximum Column length

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

// Commands supported by the LCD display
typedef enum
{
    kLcdCmdDisplayOn = 0,
    kLcdCmdDisplayOff,
    kLcdCmdSetCursor,
    kLcdCmdCursorHome,
    kLcdCmdUnderlineCursorOn,
    kLcdCmdUnderlineCursorOff,
    kLcdCmdMoveCursorLeftOnePlace,
    kLcdCmdMoveCursorRightOnePlace,
    kLcdCmdBlinkingCursorOn,
    kLcdCmdBlinkingCursorOff,
    kLcdCmdBackspace,
    kLcdCmdClearScreen,
    kLcdCmdSetContrast,
    kLcdCmdSetBacklightBrightness,
    kLcdCmdLoadCustomCharacter,
    kLcdCmdMoveDisplayOnePlaceToTheLeft,
    kLcdCmdMoveDisplayOnePlaceToTheRight,
    kLcdCmdChangeRS232BaudRate,
    kLcdCmdChangeI2CAddress,
    kLcdCmdDisplayFirmwareVersionNumber,
    kLcdCmdDisplayRS232BaudRate,
    kLcdCmdDisplayI2CAddress,
} tLcdCmd;

// Command description
typedef struct
{
    uint8_t     command;
    uint8_t     padding;
    uint16_t    executionDuration;
} tLcdCmdDesc;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
ALT_I2C_DEV_t*          deviceHandle_l;

// Descriptions for all supported commands
static tLcdCmdDesc      lcdCommands_l[] =
{
    {0x41, 0, 100},     // LCD_COMMAND_DISPLAY_ON,
    {0x42, 0, 100},     // LCD_COMMAND_DISPLAY_OFF,
    {0x45, 1, 100},     // LCD_COMMAND_SET_CURSOR,
    {0x46, 0, 1500},    // LCD_COMMAND_CURSOR_HOME,
    {0x47, 0, 1500},    // LCD_COMMAND_UNDERLINE_CURSOR_ON,
    {0x48, 0, 1500},    // LCD_COMMAND_UNDERLINE_CURSOR_OFF,
    {0x49, 0, 100},     // LCD_COMMAND_MOVE_CURSOR_LEFT_ONE_PLACE,
    {0x4A, 0, 100},     // LCD_COMMAND_MOVE_CURSOR_RIGHT_ONE_PLACE,
    {0x4B, 0, 100},     // LCD_COMMAND_BLINKING_CURSOR_ON,
    {0x4C, 0, 100},     // LCD_COMMAND_BLINKING_CURSOR_OFF,
    {0x4E, 0, 100},     // LCD_COMMAND_BACKSPACE,
    {0x51, 0, 1500},    // LCD_COMMAND_CLEAR_SCREEN,
    {0x52, 1, 500},     // LCD_COMMAND_SET_CONTRAST,
    {0x53, 1, 100},     // LCD_COMMAND_SET_BACKLIGHT_BRIGHTNESS,
    {0x54, 9, 200},     // LCD_COMMAND_LOAD_CUSTOM_CHARACTER,
    {0x55, 0, 100},     // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_LEFT,
    {0x56, 0, 100},     // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_RIGHT,
    {0x61, 1, 3000},    // LCD_COMMAND_CHANGE_RS_232_BAUD_RATE,
    {0x62, 1, 3000},    // LCD_COMMAND_CHANGE_I2C_ADDRESS,
    {0x70, 0, 4000},    // LCD_COMMAND_DISPLAY_FIRMWARE_VERSION_NUMBER,
    {0x71, 0, 10000},   // LCD_COMMAND_DISPLAY_RS_232_BAUD_RATE,
    {0x72, 0, 4000},    // LCD_COMMAND_DISPLAY_I2C_ADDRESS,
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static inline ALT_STATUS_CODE sendLcdCommand(ALT_I2C_DEV_t* deviceHdl_p,
                                             tLcdCmd command_p,
                                             const uint8_t* pArg_p);
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the LCD

This function writes a sequence of initialization parameters to the LCD.
*/
//------------------------------------------------------------------------------
void lcdl_init(void)
{
    int                     ret = 0;
    ALT_I2C_MASTER_CONFIG_t cfg;
    uint32_t                speed;

    // Init I2C module
    if (alt_i2c_init(ALT_I2C_I2C0, deviceHandle_l) != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }

    if (alt_i2c_enable(deviceHandle_l) != ALT_E_SUCCESS)       // Enable I2C module
    {
        ret = -1;
        goto Exit;
    }

    if (alt_i2c_master_config_get(deviceHandle_l, &cfg) != ALT_E_SUCCESS) // Configure I2C module
    {
        ret = -1;
        goto Exit;
    }

    if (alt_i2c_master_config_speed_get(deviceHandle_l, &cfg, (uint32_t*)&speed) != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }
    TRACE("LCD INFO: Current I2C speed = %d Hz.\n", (int)speed);

    if (alt_i2c_master_config_speed_set(deviceHandle_l, &cfg, LCD_I2C_SPEED) != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }

    if (alt_i2c_master_config_speed_get(deviceHandle_l, &cfg, (uint32_t*)&speed) != ALT_E_SUCCESS)
    {
        ret = -1;
        goto Exit;
    }
    else
    {
        TRACE("LCD INFO: New I2C speed = %d Hz.\n", (int)speed);
        cfg.addr_mode = ALT_I2C_ADDR_MODE_7_BIT;
        cfg.restart_enable = ALT_E_TRUE;

        if (alt_i2c_master_config_set(deviceHandle_l, &cfg) != ALT_E_SUCCESS)
        {
            ret = -1;
            goto Exit;
        }

        if (alt_i2c_sda_hold_time_set(deviceHandle_l, 8) != ALT_E_SUCCESS)
        {
            ret = -1;
            goto Exit;
        }

        if (alt_i2c_master_target_set(deviceHandle_l, LCD_I2C_ADDRESS) != ALT_E_SUCCESS)    // Set target display I2C address
        {
            ret = -1;
            goto Exit;
        }

        if (sendLcdCommand(deviceHandle_l, kLcdCmdDisplayOn, NULL) != ALT_E_SUCCESS)        // Turn display on
        {
            ret = -1;
            goto Exit;
        }

        if (sendLcdCommand(deviceHandle_l, kLcdCmdBlinkingCursorOn, NULL) != ALT_E_SUCCESS) // Turn cursor on
        {
            ret = -1;
            goto Exit;
        }
        else
            ret = 0;
    }

 Exit:
    if (ret != 0)
    {
        TRACE("LCD ERR: Initialization failed!!\n");
    }
}

//------------------------------------------------------------------------------
/**
\brief  Exit the LCD instance

This function exits the LCD instance.
*/
//------------------------------------------------------------------------------
void lcdl_exit(void)
{
    lcdl_clear();
    sendLcdCommand(deviceHandle_l, kLcdCmdDisplayOff, NULL);
    sendLcdCommand(deviceHandle_l, kLcdCmdBlinkingCursorOff, NULL);
    alt_i2c_disable(deviceHandle_l);
    alt_i2c_uninit(deviceHandle_l);
}

//------------------------------------------------------------------------------
/**
\brief  Clear the LCD

This function clears all lines of the display.
*/
//------------------------------------------------------------------------------
void lcdl_clear(void)
{
    // Clear screen
    if (sendLcdCommand(deviceHandle_l, kLcdCmdClearScreen, NULL) != ALT_E_SUCCESS)
    {
        TRACE("LCD ERR: Failed to clear screen\n");
    }
}

//------------------------------------------------------------------------------
/**
\brief  Change to specified line

Changes to specified line of the LCD.

\param[in]      line_p              Specifies the line

\return The function returns 0 if the line is changed successfully, -1 otherwise.
*/
//------------------------------------------------------------------------------
int lcdl_changeToLine(unsigned int line_p)
{
    uint8_t param;

    if (line_p < 2)
        param = LCD_POS_1ST_LINE;
    else
        param = LCD_POS_2ND_LINE;

    if (sendLcdCommand(deviceHandle_l, kLcdCmdSetCursor, &param) != ALT_E_SUCCESS)
        return -1;
    else
        return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Print text to the LCD

Writes text to the LCD currently selected.

\param[in]      sText_p             The text to print
*/
//------------------------------------------------------------------------------
void lcdl_printText(const char* sText_p)
{
    ALT_STATUS_CODE halRet = ALT_E_SUCCESS;
    size_t          txtLen = strlen(sText_p);
    const char      padTxt = ' ';

    for (size_t i = 0; i < LCDL_COLUMN; i++)
    {
        if (i < txtLen)
            halRet = alt_i2c_master_transmit(deviceHandle_l,
                                             &sText_p[i],
                                             1,
                                             ALT_E_FALSE,
                                             ALT_E_TRUE);
        else
            halRet = alt_i2c_master_transmit(deviceHandle_l,
                                             &padTxt,
                                             1,
                                             ALT_E_FALSE,
                                             ALT_E_TRUE);

        if (halRet != ALT_E_SUCCESS)
            break;
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Send command to LCD display

The function sends a I2C command to the LCD module.

\param[in,out]  deviceHdl_p         I2C device
\param[in]      command_p           Opcode of command to be sent
\param[in]      pArg_p              Command parameters or NULL for no parameters

 \return Returns an ALT_STATUS_CODE error code.
*/
//------------------------------------------------------------------------------
static inline ALT_STATUS_CODE sendLcdCommand(ALT_I2C_DEV_t* deviceHdl_p,
                                             tLcdCmd command_p,
                                             const uint8_t* pArg_p)
{
    ALT_STATUS_CODE halRet = ALT_E_SUCCESS;
    tLcdCmdDesc     lcdCommandDesc = lcdCommands_l[(int)command_p];
    uint8_t         data[10];
    uint8_t         dataLen = 0;

    data[dataLen++] = LCD_ESCAPE_CHAR;
    data[dataLen++] = lcdCommandDesc.command;
    for (uint8_t i = 0; i < lcdCommandDesc.padding; i++)
        data[dataLen++] = pArg_p[i];

    halRet = alt_i2c_master_transmit(deviceHdl_p,
                                     data,
                                     dataLen,
                                     ALT_E_FALSE,
                                     ALT_E_TRUE);

    return halRet;
}

/// \}
