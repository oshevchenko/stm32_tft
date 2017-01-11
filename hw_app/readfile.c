/***********************************************************************
* Code listing from "Advanced Linux Programming," by CodeSourcery LLC  *
* Copyright (C) 2001 by New Riders Publishing                          *
* See COPYRIGHT for license information.                               *
***********************************************************************/

#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#define ARRAY_SIZE 10*1024
#define USB_ARRAY_SIZE 5


char* read_write_file (const char* filename, size_t length)
{
  char* buffer;
  int fd; 
  static int u_random_fd = -1;
  ssize_t bytes_read;
  unsigned random;
  struct pollfd pfds[2];
  unsigned cnt = 0;

  /* Allocate the buffer.  */
  buffer = (char*) malloc (ARRAY_SIZE);
  if (buffer == NULL)
    return NULL;
  /* Open the file.  */
  fd = open (filename, O_RDWR);
  if (fd == -1) {
    /* open failed.  Deallocate buffer before returning.  */
    printf ("Open  %s failed!\n", filename);
    free (buffer);
    return NULL;
  }



  if (u_random_fd == -1) {
    /* Open /dev/random.  Note that we're assuming that /dev/random
       really is a source of random bits, not a file full of zeros
       placed there by an attacker.  */
    u_random_fd = open ("/dev/urandom", O_RDONLY);
    /* If we couldn't open /dev/random, give up.  */
    if (u_random_fd == -1) {
      printf ("Open  /dev/random failed!\n");
      return NULL;
    }
  }

  printf ("Now read from random!\n");


  /* Read an integer from /dev/random.  */
  if (read (u_random_fd, buffer, ARRAY_SIZE) != ARRAY_SIZE)
    return NULL;


  printf ("Now write to device!\n");

  pfds[0].fd = 0;
  pfds[0].events = POLLIN;
  pfds[1].fd = fd;
  pfds[1].events = POLLIN;

  write(fd, buffer, ARRAY_SIZE);


//  printf ("Now read from device!\n");
  do
  {
    poll(pfds, 2, 0);
    cnt++;
    if (pfds[1].revents & POLLIN) {
      bytes_read = read (fd, buffer, 3);
      if (bytes_read != 3) {
        printf ("Read from device failed!\n");
        /* read failed.  Deallocate buffer and close fd before returning.  */
        free (buffer);
        close (fd);
        return NULL;
      }
      break;
    }
  } while(1);

  /* Read the data.  */
  /* Everything's fine.  Close the file and return the buffer.  */
  close (fd);
  printf ("cnt: %d\n", cnt);
  printf ("min: %d\n", (unsigned char) buffer[0]);
  printf ("max: %d\n", (unsigned char) buffer[1]);
  printf ("avg: %d\n", (unsigned char) buffer[2]);
  free(buffer);
  return NULL;
}


char* write_usb_file (const char* filename, size_t length)
{
  char* buffer;
  int fd; 
  static int u_random_fd = -1;
  ssize_t bytes_read;
  unsigned random;
  struct pollfd pfds[2];
  unsigned cnt = 0;

  /* Allocate the buffer.  */
  buffer = (char*) malloc (USB_ARRAY_SIZE);
  if (buffer == NULL) {
    printf ("Failed to allocate %d bytes buffer!\n", USB_ARRAY_SIZE);
    return NULL;
  }

  /* Open the file.  */
  fd = open (filename, O_RDWR);
  if (fd == -1) {
    /* open failed.  Deallocate buffer before returning.  */
    printf ("Open %s failed!\n", filename);
    goto exit;
  }

  if (u_random_fd == -1) {
    /* Open /dev/urandom.  Note that we're assuming that /dev/random
       really is a source of random bits, not a file full of zeros
       placed there by an attacker.  */
    u_random_fd = open ("/dev/urandom", O_RDONLY);
    /* If we couldn't open /dev/random, give up.  */
    if (u_random_fd == -1) {
      printf ("Open  /dev/random failed!\n");
      goto exit;
    }
  }

  /* Read an integer from /dev/random.  */
  // printf ("Now read from random!\n");
  if (read (u_random_fd, buffer, USB_ARRAY_SIZE) != USB_ARRAY_SIZE) {
    printf ("Error while reading random numbers.\n");
    goto exit;
  }
    
  printf ("Now write to USB device to toggle LED ... ");
  if (write(fd, buffer, USB_ARRAY_SIZE) != USB_ARRAY_SIZE) {
    printf ("Error: %s.\n", strerror( errno ));
    goto exit;
  }

  printf ("OK!\n");
exit:
  free(buffer);
  return NULL;
}
