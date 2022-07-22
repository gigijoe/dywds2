/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef RK120_07_H_  
#define RK120_07_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void RK120_07_Init(void);
void RK120_07_Run(void);

uint16_t RK120_07_WindDirection();
float RK120_07_WindSpeed();

#ifdef __cplusplus
}
#endif

#endif