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
#include <stdint.h>
#include <string.h>

#define ARRAY_SIZE 10*1024
#define USB_ARRAY_SIZE 64


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
typedef struct
{
  uint16_t horz;
  uint16_t vert;
  uint32_t pixelTotal;
  uint8_t  CB[56];
  uint8_t  ReservedForAlign;
}
USBD_MSC_BOT_CBWTypeDef;


#pragma pack(push, 1)

// typedef struct tagBITMAPINFOHEADER
// {
//     uint32_t biSize;  //specifies the number of bytes required by the struct
//     LONG biWidth;  //specifies width in pixels
//     LONG biHeight;  //species height in pixels
//     uint16_t biPlanes; //specifies the number of color planes, must be 1
//     uint16_t biBitCount; //specifies the number of bit per pixel
//     uint32_t biCompression;//spcifies the type of compression
//     uint32_t biSizeImage;  //size of image in bytes
//     LONG biXPelsPerMeter;  //number of pixels per meter in x axis
//     LONG biYPelsPerMeter;  //number of pixels per meter in y axis
//     uint32_t biClrUsed;  //number of colors used by th ebitmap
//     uint32_t biClrImportant;  //number of colors that are important
// }BITMAPINFOHEADER;

typedef struct tagBITMAPFILEHEADER
{
    uint16_t bfType;  //specifies the file type
    uint32_t bfSize;  //specifies the size in bytes of the bitmap file
    uint16_t bfReserved1;  //reserved; must be 0
    uint16_t bfReserved2;  //reserved; must be 0
    uint32_t bOffBits;  //species the offset in bytes from the bitmapfileheader to the bitmap bits
}BITMAPFILEHEADER;

#pragma pack(pop)

char* write_usb_file (const char* filename, const char* bmp_filename, size_t length)
{
  char* buffer = NULL;
  char* pPixel = NULL;
  char* pBufferEnd = NULL;
  int fd; 
  FILE *fd_bmp = NULL;
  ssize_t bytes_read;
  unsigned random;
  struct pollfd pfds[2];
  unsigned cnt = 0;
  unsigned long sz_trans = 0;
  USBD_MSC_BOT_CBWTypeDef msc_bot;
  BITMAPFILEHEADER bitmapFileHeader;
  /* Open the file.  */
  fd = open (filename, O_RDWR);
  if (fd == -1) {
    /* open failed.  Deallocate buffer before returning.  */
    printf ("Open %s failed!\n", filename);
    goto exit;
  }

//  fd_bmp = fopen ("/mnt/hgfs/workspace-hp/photo_shchengen_visa2.bmp", "rb");
  fd_bmp = fopen (bmp_filename, "rb");  
  if (fd_bmp == NULL) {
    printf ("Open bmp failed!\n");
    goto exit;
  }

  // printf ("Now read from random!\n");
  cnt = fread (&bitmapFileHeader, sizeof(BITMAPFILEHEADER),1,fd_bmp);
  if (cnt != 1) {
    printf ("Error while reading bmp image header! 0x%04X %d\n", bitmapFileHeader.bfType,cnt);
    goto exit;
  }

  //verify that this is a bmp file by check bitmap id
  if (bitmapFileHeader.bfType !=0x4D42)
  {
    printf ("Wrong bmp header!\n");
    goto exit;
  }  

  /* Allocate the buffer.  */
  buffer = (char*) malloc (bitmapFileHeader.bfSize);
  if (buffer == NULL) {
    printf ("Failed to allocate %d bytes buffer!\n", bitmapFileHeader.bfSize);
    return NULL;
  }
  fseek(fd_bmp, 0L, SEEK_SET);
  cnt = fread (buffer, bitmapFileHeader.bfSize,1,fd_bmp);
  if ( cnt != 1) {
    printf ("Error while reading bmp image!\n");
    goto exit;
  }
  pPixel = buffer + bitmapFileHeader.bOffBits;
  pBufferEnd = buffer + bitmapFileHeader.bfSize;


  msc_bot.horz = 0;
  msc_bot.vert = 0;
  msc_bot.pixelTotal = 320*240;
  memcpy(&msc_bot.CB[0], pPixel, 56);
  pPixel+=56;

  printf ("Buffer:\n");
  for( cnt = 0; cnt < USB_ARRAY_SIZE; cnt++)
  {
    printf ("0x%02X ", *((unsigned char*)(((unsigned char*) &msc_bot)) + cnt)) ;
  }
  printf ("\n");
  printf ("Now write to USB device to toggle LED ... \n");
  if (write(fd, (unsigned char*) &msc_bot, USB_ARRAY_SIZE) != USB_ARRAY_SIZE) {
    printf ("Error: %s.\n", strerror( errno ));
    goto exit;
  }

  while(pPixel < pBufferEnd){
    sz_trans = pBufferEnd - pPixel;
//    sz_trans = sz_trans > USB_ARRAY_SIZE ? USB_ARRAY_SIZE : sz_trans;
    sz_trans = sz_trans > 2048 ? 2048 : sz_trans;
    if (write(fd, pPixel, sz_trans) != sz_trans) {
      printf ("Error: %s.\n", strerror( errno ));
      goto exit;
    }
    pPixel+=sz_trans;
  }


  printf ("OK!\n");
exit:
  if (fd_bmp != NULL) {
    fclose(fd_bmp);
  }
  if (fd != -1) {
    close(fd);
  }
  if (buffer != NULL) {
    free(buffer);
  }
  return NULL;
}
