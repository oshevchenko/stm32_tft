/***********************************************************************
* Code listing from "Advanced Linux Programming," by CodeSourcery LLC  *
* Copyright (C) 2001 by New Riders Publishing                          *
* See COPYRIGHT for license information.                               *
***********************************************************************/

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

/* The name of this program.  */
const char* program_name;

/* Prints usage information for this program to STREAM (typically
   stdout or stderr), and exit the program with EXIT_CODE.  Does not
   return.  */

void print_usage (FILE* stream, int exit_code)
{
  fprintf (stream, "Usage:  %s options [ inputfile ... ]\n", program_name);
  fprintf (stream,
           "  -h  --help             Display this usage information.\n"
           "  -i  --input filename   Read from file.\n"
           "  -v  --verbose          Print verbose messages.\n");
  exit (exit_code);
}

/* Main program entry point.  ARGC conains number of argument list
   elements; ARGV is an array of pointers to them.  */

int main (int argc, char* argv[])
{
  int next_option;

  /* A string listing valid short options letters.  */
  const char* const short_options = "hi:u:v";
  /* An array describing valid long options.  */
  const struct option long_options[] = {
    { "help",     0, NULL, 'h' },
    { "input",    1, NULL, 'i' },
    { "uwrite",    1, NULL, 'u' },
    { "verbose",  0, NULL, 'v' },
    { NULL,       0, NULL, 0   }   /* Required at end of array.  */
  };

  /* The name of the file to receive program output, or NULL for
     standard output.  */
  const char* input_filename = NULL;
  /* Whether to display verbose messages.  */
  int verbose = 0;

  /* Remember the name of the program, to incorporate in messages.
     The name is stored in argv[0].  */
  program_name = argv[0];

  do {
    next_option = getopt_long (argc, argv, short_options,
                               long_options, NULL);
    switch (next_option)
    {
    case 'h':   /* -h or --help */
      /* User has requested usage information.  Print it to standard
         output, and exit with exit code zero (normal termination).  */
      print_usage (stdout, 0);

    case 'i':   /* -i or --input */
      /* This option takes an argument, the name of the output file.  */
      input_filename = optarg;
      read_write_file(input_filename, 0);
      break;
    case 'u':   /**/
      /* This option takes an argument, the name of the output file.  */
      input_filename = optarg;
      write_usb_file(input_filename, 0);
      break;

    case 'v':   /* -v or --verbose */
      verbose = 1;
      break;

    case '?':   /* The user specified an invalid option.  */
      /* Print usage information to standard error, and exit with exit
         code one (indicating abonormal termination).  */
      print_usage (stderr, 1);

    case -1:    /* Done with options.  */
      break;

    default:    /* Something else: unexpected.  */
      abort ();
    }
  }
  while (next_option != -1);

  /* Done with options.  OPTIND points to first non-option argument.
     For demonstration purposes, print them if the verbose option was
     specified.  */
  if (verbose) {
    int i;
    for (i = optind; i < argc; ++i) 
      printf ("Argument: %s\n", argv[i]);
  }

  /* The main program goes here.  */

  return 0;
}
