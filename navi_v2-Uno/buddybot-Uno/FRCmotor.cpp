 /*   This file is part of ArduRIO.

    ArduRIO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ArduRIO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ArduRIO.  If not, see <http://www.gnu.org/licenses/>. */
 
 #include "FRCmotor.h"
 #include "Arduino.h"
 extern int gamemode;
#if defined __AVR_ATmega328P__
void FRCmotor::SetPort (int pin){
    this->pin=pin;
    if(pin == 9 || pin == 10){
    TCCR1B = TCCR1B & 0b11111000 | 0x04;
    }
    if(pin == 11 || pin == 3){
    TCCR2B = TCCR2B & 0b11111000 | 0x06;
    }
    if(pin == 5 || pin == 6){
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR0B = TCCR0B & 0b11111000 | 0x04;
    }
}

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
void FRCmotor::SetPort (int pin){
    this->pin=pin;
    if(pin == 12 || pin == 11){
    TCCR1B = TCCR1B & 0b11111000 | 0x04;
    }
    if(pin == 5 || pin == 3 || pin == 2){
    TCCR3B = TCCR3B & 0b11111000 | 0x04;
    }
    if(pin == 8 || pin == 7 || pin == 6){
    TCCR4B = TCCR4B & 0b11111000 | 0x04;
    }
    if(pin == 10 || pin == 9){
    TCCR2B = TCCR2B & 0b11111000 | 0x06;
    }
    if(pin == 13 || pin == 4){
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR0B = TCCR0B & 0b11111000 | 0x04;
    }
}

#endif
void FRCmotor::Set (float power){
if(gamemode){
    power /= 100;
    value = 47 + power*27; 
    if(value > 74){
        value = 74;
        }
        else if(value < 20){
            value = 20;
            }
        }
     else{//if gamemode = 0
        value = 0;
    }
    analogWrite(pin,value);
}
