//#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include "usb_device.h"
#include <unistd.h> 
#include <string.h>
#include "src/config.h"
#include "src/microrl.h"
#include "stm32_microrl_misc.h"
#include "event_queue.h"

extern void USBD_CDC_TxAlways(const uint8_t *buf, uint32_t len);
static microrl_t rl;
static microrl_t * prl = &rl;
static unsigned curr_cmd_param;
static enum {COMMAND_EMPTY, COMMAND_WIDTH, COMMAND_OFFSET, COMMAND_HV_ON, COMMAND_HV_OFF} curr_cmd = COMMAND_EMPTY;

//*****************************************************************************
//dummy function, no need on linux-PC
void init (void){
	// Console terminal init
	// call init with ptr to microrl instance and print callback
	microrl_init (prl, print);
	// set callback for execute
	microrl_set_execute_callback (prl, execute);
#ifdef _USE_COMPLETE
	// set callback for completion
	microrl_set_complete_callback (prl, complet);
#endif
	// set callback for Ctrl+C
	microrl_set_sigint_callback (prl, sigint);

};

void stm32_microrl_insert_char(int ch)
{
	microrl_insert_char (prl, ch);
}



//*****************************************************************************
// print callback for microrl library
void print (const char * str)
{
//	int i = 0;
	USBD_CDC_TxAlways((uint8_t*)str, (uint32_t)strlen(str));
}

// definition commands word
#define _CMD_HELP  "help"
#define _CMD_CLEAR "clear"
#define _CMD_LIST  "list"
#define _CMD_NAME  "name"
#define _CMD_VER   "version"
// sub commands for version command
	#define _SCMD_MRL  "microrl"
	#define _SCMD_DEMO "demo"
#define _CMD_WIDTH "width"
#define _CMD_OFFSET "offset"
#define _CMD_HV  "hv" //
// sub commands for HV command
	#define _SCMD_SWITCH_ON  "on"
	#define _SCMD_SWITCH_OFF "off"

#define _NUM_OF_CMD 8
#define _NUM_OF_VER_SCMD 2
#define _NUM_OF_SWITCH_SCMD 2

//available  commands
static char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_LIST, _CMD_NAME, _CMD_VER, _CMD_HV};
// version subcommands
static char * ver_keyworld [] = {_SCMD_MRL, _SCMD_DEMO};
static char * switch_keyworld [] = {_SCMD_SWITCH_ON, _SCMD_SWITCH_OFF};

// array for comletion
static char * compl_world [_NUM_OF_CMD + 1];

// 'name' var for store some string
#define _NAME_LEN 8
char name [_NAME_LEN];
int val;



//*****************************************************************************
void print_help ()
{
	print ("--------------------------------------\n\r");
	print ("--       EMAT Thickness Meter       --\n\r");
	print ("--------------------------------------\n\r");
	print ("Use TAB key for completion\n\rCommand:\n\r");
	print ("\tversion {microrl | demo} - print version of microrl lib or version of this demo src\n\r");
	print ("\thelp  - this message\n\r");
	print ("\tclear - clear screen\n\r");
	print ("\tlist  - list all commands in tree\n\r");
	print ("\tname [string] - print 'name' value if no 'string', set name value to 'string' if 'string' present\n\r");
	print ("\toffset {offset} - set pulse offset, us\n\r");
	print ("\twidth {width} - set pulse width, us\n\r");
	print ("\thv {on | off}- +300V power supply on off\n\r");
}

//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], _CMD_HELP) == 0) {
//			print ("microrl library based shell v 1.0\n\r");
			print_help ();        // print help
		} else if (strcmp (argv[i], _CMD_NAME) == 0) {
			if ((++i) < argc) { // if value preset
				if (strlen (argv[i]) < _NAME_LEN) {
					strcpy (name, argv[i]);
				} else {
					print ("name value too long!\n\r");
				}
			} else {
				print (name);
				print ("\n\r");
			}
		} else if (strcmp (argv[i], _CMD_VER) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], _SCMD_DEMO) == 0) {
					print ("\n\rdemo v 1.0\n\r");
				} else if (strcmp (argv[i], _SCMD_MRL) == 0) {
					print ("\n\rmicrorl v 1.2\n\r");
				} else {
					print ((char*)argv[i]);
					print (" wrong argument, see help\n\r");
				}
			} else {
				print ("version needs 1 parametr, see help\n\r");
			}
		} else if (strcmp (argv[i], _CMD_CLEAR) == 0) {
			print ("\033[2J");    // ESC seq for clear entire screen
			print ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], _CMD_LIST) == 0) {
			print ("available command:\n\r");// print all command per line
			for (int i = 0; i < _NUM_OF_CMD; i++) {
				print ("\t");
				print (keyworld[i]);
				print ("\n\r");
			}
		} else if (strcmp (argv[i], _CMD_WIDTH) == 0) {
			if (++i < argc) {
				char* endptr;
				curr_cmd_param = strtol(argv[i],&endptr,10);
				curr_cmd = COMMAND_WIDTH;
				print ("\n\r");
			} else {
				print ("width needs 1 parametr, see help\n\r");
			}
		} else if (strcmp (argv[i], _CMD_OFFSET) == 0) {
			if (++i < argc) {
				char* endptr;
				curr_cmd_param = strtol(argv[i],&endptr,10);
				curr_cmd = COMMAND_OFFSET;
				print ("\n\r");
			} else {
				print ("offset needs 1 parametr, see help\n\r");
			}
		} else if (strcmp (argv[i], _CMD_HV) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], _SCMD_SWITCH_ON) == 0) {
					curr_cmd = COMMAND_HV_ON;
					print ("\n\r");
				} else if (strcmp (argv[i], _SCMD_SWITCH_OFF) == 0) {
					curr_cmd = COMMAND_HV_OFF;
					print ("\n\r");
				} else {
					print ((char*)argv[i]);
					print (" wrong argument, see help\n\r");
				}
			} else {
				print ("version needs 1 parametr, see help\n\r");
			}
		} else {
			print ("command: '");
			print ((char*)argv[i]);
			print ("' Not found.\n\r");
		}
		i++;
	}
	return 0;
}

#ifdef _USE_COMPLETE
//*****************************************************************************
// completion callback for microrl library
char ** complet (int argc, const char * const * argv)
{
	int j = 0;

	compl_world [0] = NULL;

	// if there is token in cmdline
	if (argc == 1) {
		// get last entered token
		char * bit = (char*)argv [argc-1];
		// iterate through our available token and match it
		for (int i = 0; i < _NUM_OF_CMD; i++) {
			// if token is matched (text is part of our token starting from 0 char)
			if (strstr(keyworld [i], bit) == keyworld [i]) {
				// add it to completion set
				compl_world [j++] = keyworld [i];
			}
		}
	} else if ((argc > 1) && (strcmp (argv[0], _CMD_VER)==0)) { // if command needs subcommands
		// iterate through subcommand for command _CMD_VER array
		for (int i = 0; i < _NUM_OF_VER_SCMD; i++) {
			if (strstr (ver_keyworld [i], argv [argc-1]) == ver_keyworld [i]) {
				compl_world [j++] = ver_keyworld [i];
			}
		}
	} else if ((argc > 1) && (strcmp (argv[0], _CMD_HV)==0)) { // if command needs subcommands
		// iterate through subcommand for command _CMD_SWITCH array
		for (int i = 0; i < _NUM_OF_SWITCH_SCMD; i++) {
			if (strstr (switch_keyworld [i], argv [argc-1]) == switch_keyworld [i]) {
				compl_world [j++] = switch_keyworld [i];
			}
		}
	} else { // if there is no token in cmdline, just print all available token
		for (; j < _NUM_OF_CMD; j++) {
			compl_world[j] = keyworld [j];
		}
	}

	// note! last ptr in array always must be NULL!!!
	compl_world [j] = NULL;
	// return set of variants
	return compl_world;
}
#endif

//*****************************************************************************
void sigint (void)
{
	print ("^C catched!\n\r");
}

void TERM_Task(void)
{
	eq_queue_param_u param;

	switch (curr_cmd){
	case COMMAND_WIDTH:
		param.uiParam = curr_cmd_param;
		EQ_PutEventParam(CMD_WIDTH, param);
		break;
	case COMMAND_OFFSET:
		param.uiParam = curr_cmd_param;
		EQ_PutEventParam(CMD_OFFSET, param);
		break;
	case COMMAND_HV_ON:
		EQ_PutEvent(CMD_HV_ON);
		break;
	case COMMAND_HV_OFF:
		EQ_PutEvent(CMD_HV_OFF);
		break;
	}
	curr_cmd = COMMAND_EMPTY;
}


