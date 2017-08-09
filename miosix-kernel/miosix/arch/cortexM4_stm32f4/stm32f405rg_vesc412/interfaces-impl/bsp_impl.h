/***************************************************************************
 *   Copyright (C) 2012 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

/***********************************************************************
* bsp_impl.h Part of the Miosix Embedded OS.
* Board support package, this file initializes hardware.
************************************************************************/

#ifndef BSP_IMPL_H
#define BSP_IMPL_H

#ifndef VESC412
    #define VESC412
#endif

#include "config/miosix_settings.h"
#include "interfaces/gpio.h"
#include "drivers/stm32_hardware_rng.h"

namespace miosix {

/**
\addtogroup Hardware
\{
*/

/**
 * \internal
 * used by the ledOn() and ledOff() implementation
 */
typedef Gpio<GPIOC_BASE,4> _greenLED;

inline void greenLED_on()
{
    _greenLED::high();
}

inline void greenLED_off()
{
    _greenLED::low();
}

/*inline void greenLED_toggle()
{
    _greenLED::toggle();
}*/

typedef Gpio<GPIOC_BASE,5> _redLED;

inline void redLED_on()
{
    _redLED::high();
}

inline void redLED_off()
{
    _redLED::low();
}

/*inline void redLED_toggle()
{
    _redLED::toggle();
}*/

/**
 * Polls the SD card sense GPIO.
 * But the VESC board doesn't have an SD card.
 * \return true. As there's no SD card sense switch, let's pretend that
 * the card is present.
 */
inline bool sdCardSense() { return true; }

/**
\}
*/

};//namespace miosix

#endif //BSP_IMPL_H
