/* Copyright (C) 2015, Santiago F. Maudet
 * This file is part of char01 module.
 *
 * char01 module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * char01 module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* !\brief
*   This  is a user application to read data from the module buffer.
*   USAGE: ./test_module /dev/char_01
*/



#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <string.h>

#define BUF_SIZE 32
#define MAX_LOOPS 10

int main (int argc, char ** argv){

  int fd;

  int loops = 0;

  int status, nbytes;
  
  char data[BUF_SIZE];

  if(argc < 2){
    perror("usage: ./test_module /dev/[device_name]");
    return -1;
  }

  fd = open(argv[1],O_RDWR);

  if( fd == -1){
    perror("Error openning device");
    return -1;
  }
 
  while(loops < MAX_LOOPS) {
  
    fgets(data,BUF_SIZE,stdin);
    printf("String received: %s", data);
    
    status = write(fd,data,strlen(data));
    
    if(status == -1) {
    	perror("Could not write to my_chardevice :(\n");
    	break;
    }
    
    if(status == 0) {
      printf("Buffer is already full. Exiting...\n ");	
    	break;
    }
        
    printf(" %lu bytes copied\n",strlen(data));
    loops++;
    pause;
  }  
  
  close(fd);
  
  return status;
}
