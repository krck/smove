//  SerialService.h
/*************************************************************************************
 *  smove   -   a lightweight, G-Code based, CNC motion control software             *
 *-----------------------------------------------------------------------------------*
 *  Copyright (c) 2016, Peter Baumann                                                *
 *  All rights reserved.                                                             *
 *                                                                                   *
 *  Redistribution and use in source and binary forms, with or without               *
 *  modification, are permitted provided that the following conditions are met:      *
 *    1. Redistributions of source code must retain the above copyright              *
 *       notice, this list of conditions and the following disclaimer.               *
 *    2. Redistributions in binary form must reproduce the above copyright           *
 *       notice, this list of conditions and the following disclaimer in the         *
 *       documentation and/or other materials provided with the distribution.        *
 *    3. Neither the name of the organization nor the                                *
 *       names of its contributors may be used to endorse or promote products        *
 *       derived from this software without specific prior written permission.       *
 *                                                                                   *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  *
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
 *  DISCLAIMED. IN NO EVENT SHALL PETER BAUMANN BE LIABLE FOR ANY                    *
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       *
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      *
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     *
 *                                                                                   *
 *************************************************************************************/

#ifndef SERIALSERVICE_H
#define SERIALSERVICE_H

#include <string>
#include <iostream>
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <termios.h>  // POSIX terminal control definitions

#include "../Config.h"


class SerialService {

private:
    const std::string port;
    const int baud;
    int fd;

public:
    SerialService(const std::string& portName, int baudRate) : port(portName), baud(baudRate) {
        serial_init();
    }

    ~Serial() { close_port(); }


    int write_byte(uint8_t c) {
        const size_t n = write(this->fd, &c, 1);
        if(n != 1) return -1;
        else return 0;
    }

    int write_string(const std::string& data) {
        const size_t n = write(this->fd, data.c_str(), data.size());
        if(n != (int)data.size()) { std::cerr <<"ERROR: Writing whole string\n"; return -1; }
        else { return 0; }
    }

    //    int serialport_read_until(char* buf, char until, int buf_max, int timeout) {
    //        char b[1];  // read expects an array, so we give it a 1-byte array
    //        int i=0;
    //        do {
    //            int n = read(fd, b, 1);  // read a char at a time
    //            if( n==-1) return -1;    // couldn't read
    //            if( n==0 ) {
    //                usleep( 1 * 1000 );  // wait 1 msec try again
    //                timeout--;
    //                if( timeout==0 ) return -2;
    //                continue;
    //            }
    //    #ifdef SERIALPORTDEBUG
    //            //printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
    //    #endif
    //            buf[i] = b[0];
    //            i++;
    //        } while( b[0] != until && i < buf_max && timeout>0 );

    //        buf[i] = 0;  // null terminate the string
    //        return 0;
    //    }

    int close_port() {
        return close(this->fd);
    }

    int flush() {
        sleep(2); //required to make flush work, for some reason
        return tcflush(this->fd, TCIOFLUSH);
    }


private:
    void serial_init() {
        struct termios toptions;

        //this->fd = open(this->port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        this->fd = open(this->port.c_str(), O_RDWR | O_NONBLOCK );

        if (fd == -1) { std::cerr <<"ERROR: Opening port\n"; return; }
        if (tcgetattr(fd, &toptions) < 0) { std::cerr <<"ERROR: Getting attributes\n"; return; }

        speed_t brate = baud; // let you override switch below if needed
        switch(baud) {
            case 4800:   brate=B4800;   break;
            case 9600:   brate=B9600;   break;
            case 14400:  brate=B14400;  break;
            case 19200:  brate=B19200;  break;
            case 28800:  brate=B28800;  break;
            case 38400:  brate=B38400;  break;
            case 57600:  brate=B57600;  break;
            case 115200: brate=B115200; break;
        }
        cfsetispeed(&toptions, brate);
        cfsetospeed(&toptions, brate);

        // 8N1
        toptions.c_cflag &= ~PARENB;
        toptions.c_cflag &= ~CSTOPB;
        toptions.c_cflag &= ~CSIZE;
        toptions.c_cflag |= CS8;
        // no flow control
        toptions.c_cflag &= ~CRTSCTS;

        //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
        toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
        toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
        toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
        toptions.c_oflag &= ~OPOST; // make raw

        // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
        toptions.c_cc[VMIN]  = 0;
        toptions.c_cc[VTIME] = 0;
        //toptions.c_cc[VTIME] = 20;

        tcsetattr(fd, TCSANOW, &toptions);
        if(tcsetattr(fd, TCSAFLUSH, &toptions) < 0) { std::cerr <<"ERROR: Setting attributes\n"; return; }
    }

};

#endif
