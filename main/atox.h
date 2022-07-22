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
#ifndef ATOX_H_  
#define ATOX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t atohex8(uint8_t *s);
float atof32(unsigned char *s);

#ifdef __cplusplus
}
#endif

#endif