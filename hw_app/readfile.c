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
#include <byteswap.h>

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
  uint8_t* buffer = NULL;
  uint8_t* p_buf = NULL;
//  char* pBufferEnd = NULL;
  int fd; 
  FILE *fd_bmp = NULL;
  ssize_t bytes_read;
  unsigned random;
  struct pollfd pfds[2];
  unsigned long cnt = 0;
  unsigned long cnt2 = 0;
  unsigned long cnt3 = 0;
  unsigned long buf_size = 0;
  unsigned long sz_trans = 0;
  USBD_MSC_BOT_CBWTypeDef msc_bot;
  BITMAPFILEHEADER bitmapFileHeader;
  uint32_t pixelTotal = 320*240;
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
#if BYTE_ORDER == BIG_ENDIAN
  bitmapFileHeader.bfType = __bswap_16(bitmapFileHeader.bfType);
  bitmapFileHeader.bfSize = __bswap_32(bitmapFileHeader.bfSize);
  bitmapFileHeader.bOffBits = __bswap_32(bitmapFileHeader.bOffBits);
#elif BYTE_ORDER == LITTLE_ENDIAN

#else
       # error "What kind of system is this?"
#endif

  //verify that this is a bmp file by check bitmap id

  if (bitmapFileHeader.bfType !=0x4D42)
  {
    printf ("Wrong bmp header! 0x%04X\n", bitmapFileHeader.bfType);
    goto exit;
  }  
  printf ("Correct bmp header! 0x%04X\n", bitmapFileHeader.bfType);


  /* Allocate the buffer.  */
  buf_size = bitmapFileHeader.bfSize - bitmapFileHeader.bOffBits + sizeof(msc_bot);
  printf ("buf_size %d bitmapFileHeader.bfSize %d bitmapFileHeader.bOffBits %d\n",buf_size, bitmapFileHeader.bfSize, bitmapFileHeader.bOffBits);
  buffer = (char*) malloc (buf_size);
  if (buffer == NULL) {
    printf ("Failed to allocate %d bytes buffer!\n", bitmapFileHeader.bfSize);
    return NULL;
  }
#if BYTE_ORDER == BIG_ENDIAN
  msc_bot.horz = __bswap_16(0);
  msc_bot.vert = __bswap_16(0);
  msc_bot.pixelTotal = __bswap_32(pixelTotal);
#elif BYTE_ORDER == LITTLE_ENDIAN
  msc_bot.horz = 0;
  msc_bot.vert = 0;
  msc_bot.pixelTotal = pixelTotal;
#else
       # error "What kind of system is this?"
#endif

  memcpy(buffer, &msc_bot, sizeof(msc_bot));
  p_buf = buffer + sizeof(msc_bot);
  printf ("Seek to %d\n", bitmapFileHeader.bOffBits);
  fseek(fd_bmp, bitmapFileHeader.bOffBits, SEEK_SET);
  cnt = fread (p_buf, bitmapFileHeader.bfSize - bitmapFileHeader.bOffBits, 1, fd_bmp);
  if ( cnt != 1) {
    printf ("Error while reading bmp image!\n");
    goto exit;
  }
#if 0
  printf ("Buffer:\n");
  for (cnt2 = 0; cnt2 <32; cnt2++)
  {
    for( cnt = 0; cnt < 16; cnt++)
    {
      printf ("0x%02X ", buffer[cnt3]);
      cnt3++;
    }
    printf ("\n");
  }
#endif
  printf ("Now write to USB device ... \n");

  cnt = 0;
  p_buf = buffer;
#if 1
  while(cnt < buf_size){
    sz_trans = buf_size - cnt;
    if (sz_trans > 32768) sz_trans = 32768;
    if (write(fd, p_buf, sz_trans) != sz_trans) {
      printf ("Error: %s.\n", strerror( errno ));
      goto exit;
    }
    p_buf += sz_trans;
    cnt += sz_trans;
  }
#else
    if (write(fd, p_buf, 512) != 512) {
      printf ("Error: %s.\n", strerror( errno ));
      goto exit;
    }
#endif

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
