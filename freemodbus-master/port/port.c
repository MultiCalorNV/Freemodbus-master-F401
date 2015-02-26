 /*
  * FreeModbus Libary: stm32f4 Port
  * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
  *
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the GNU Lesser General Public
  * License as published by the Free Software Foundation; either
  * version 2.1 of the License, or (at your option) any later version.
  *
  * This library is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * Lesser General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public
  * License along with this library; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  *
  * File: $Id: port.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
  */

/*  System includes --------------------------------*/
#include "main.h"
#include "port.h"

/*  Modbus includes ----------------------------------*/

/*  Variables ----------------------------------------*/

/*  Start implementation -----------------------------*/
void
EnterCriticalSection()
{
	__disable_irq();     /* Disable Interruptions */
}

void
ExitCriticalSection()
{
	__enable_irq();    /* Restore NVICIntEnable */
}
