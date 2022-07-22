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
#include <string.h>
#include "atox.h"

uint8_t atohex8(uint8_t *s)
{
    uint8_t value = 0;
    if(!s)
        return 0;

    if(*s >= '0' && *s <= '9')
        value = (*s - '0') << 4;
    else if(*s >= 'A' && *s <= 'F')
        value = ((*s - 'A') + 10) << 4;

    s++;

    if(*s >= '0' && *s <= '9')
        value |= (*s - '0');
    else
        value |= ((*s - 'A') + 10);

    return value;
}

union Uint32 {
    float f;
    unsigned char u[4];
};

float atof32(unsigned char *s)
{
    union Uint32 u32;

    u32.u[1] = atohex8(s);
    s+=2;
    u32.u[0] = atohex8(s);
    s+=2;
    u32.u[3] = atohex8(s);
    s+=2;
    u32.u[2] = atohex8(s);

    return u32.f;
}
