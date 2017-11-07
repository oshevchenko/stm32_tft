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
#include "stm32_dsp.h"
#include "pid_regulator.h"
#include "list.h"

extern void USBD_CDC_TxAlways(const uint8_t *buf, uint32_t len);
static microrl_t rl;
static microrl_t * prl = &rl;
static unsigned ui_curr_cmd_param;
//static int i_curr_cmd_param;
static enum {
	COMMAND_EMPTY,
	COMMAND_WIDTH,
	COMMAND_SPEED,
	COMMAND_STAB_ON,
	COMMAND_STAB_OFF,
	COMMAND_GET_ENC,
	COMMAND_UPDATE_COEFFS
} curr_cmd = COMMAND_EMPTY;

static LIST_HEAD(stat_list);
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
#define _CMD_SPEED "speed"
#define _CMD_STAB  "stab" //
#define _CMD_ENC  "enc" //
#define _CMD_K  "k" //
#define _CMD_KP  "kp" //
#define _CMD_KI  "ki" //
#define _CMD_KD  "kd" //
// sub commands for HV command
#define _SCMD_SWITCH_ON  "on"
#define _SCMD_SWITCH_OFF "off"

// sub commands for PID coefs command
#define _SCMD_DIR_X  "l"
#define _SCMD_DIR_Y  "r"



#define _NUM_OF_CMD 13
#define _NUM_OF_VER_SCMD 2
#define _NUM_OF_SWITCH_SCMD 2

//available  commands
static char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_LIST, _CMD_NAME, _CMD_VER, _CMD_STAB, _CMD_ENC};
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
	print ("-- Traffic Signs Recognition (TSR)  --\n\r");
	print ("--     Proof-of-Concept project     --\n\r");
	print ("--------------------------------------\n\r");
	print ("Use TAB key for completion\n\rCommand:\n\r");
	print ("\tversion {microrl | demo} - print version of microrl lib or version of this demo src\n\r");
	print ("\thelp  - this message\n\r");
	print ("\tclear - clear screen\n\r");
	print ("\tlist  - list all commands in tree\n\r");
	print ("\tname [string] - print 'name' value if no 'string', set name value to 'string' if 'string' present\n\r");
	print ("\tstab { on | off } - enable | disable motor speed PID regulator\n\r");
	print ("\tspeed {left right} - set motor speed.\n\r");
	print ("\tenc - get encoder statistics\n\r");
	print ("\tk - get PID regulators coeffs.\n\r");
	print ("\tkp { l | r } {val} - set P coeff. for left | right PID regulator.\n\r");
	print ("\tki { l | r } {val} - set i coeff. for left | right PID regulator.\n\r");
	print ("\tkd { l | r } {val} - set D coeff. for left | right PID regulator.\n\r");
}
#define PID_COEFS_LEN 50
void PrintPidCoefs()
{
	PID_COEFS PidCoefs;
	char cmd_buf[PID_COEFS_LEN];
	GetPidCoefs_X(&PidCoefs);
	snprintf (cmd_buf, PID_COEFS_LEN, "L: Kp=%d Ki=%d Kd=%d\n\r",
			PidCoefs.Kp, PidCoefs.Ki, PidCoefs.Kd);
	print (cmd_buf);
	GetPidCoefs_Y(&PidCoefs);
	snprintf (cmd_buf, PID_COEFS_LEN, "R: Kp=%d Ki=%d Kd=%d\n\r",
			PidCoefs.Kp, PidCoefs.Ki, PidCoefs.Kd);
	print (cmd_buf);
}

#define PID_STAT_LEN 50
void PrintPidStat()
{
	PID_REG_STAT PidStat;
	char cmd_buf[PID_STAT_LEN];
	GetPidStat(&PidStat);
	snprintf (cmd_buf, PID_STAT_LEN, "L=%05d R=%05d SL=%05d SR=%05d\n\r",
									  PidStat.enc_x, PidStat.enc_y,
									  PidStat.speed_x, PidStat.speed_y);
	print (cmd_buf);
}

void print_statistics()
{
	struct stat_module *i = NULL;
	list_for_each_entry(i, &stat_list, list) {
		i->printStatFunc();
	}
}

void SetKp_X(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_X(&PidCoefs);
	PidCoefs.Kp = param;
	SetPidCoefs_X(&PidCoefs);
}

void SetKi_X(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_X(&PidCoefs);
	PidCoefs.Ki = param;
	SetPidCoefs_X(&PidCoefs);
}

void SetKd_X(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_X(&PidCoefs);
	PidCoefs.Kd = param;
	SetPidCoefs_X(&PidCoefs);
}

void SetKp_Y(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_Y(&PidCoefs);
	PidCoefs.Kp = param;
	SetPidCoefs_Y(&PidCoefs);
}

void SetKi_Y(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_Y(&PidCoefs);
	PidCoefs.Ki = param;
	SetPidCoefs_Y(&PidCoefs);
}

void SetKd_Y(uint16_t param)
{
	PID_COEFS PidCoefs;
	GetPidCoefs_Y(&PidCoefs);
	PidCoefs.Kd = param;
	SetPidCoefs_Y(&PidCoefs);
}
static const char STR_NEED_MORE_PARAMETERS[] = "need more parameters, see help\n\r";
static const char STR_WRONG_ARGUMENT[] = "wrong argument, see help\n\r";
#define PARSE_PID_COEF(SET_FUNCTION_NAME_X, SET_FUNCTION_NAME_Y) ({\
		if (++i < argc) {                                        \
			if (strcmp (argv[i], _SCMD_DIR_X) == 0) {            \
				if (++i < argc) {                                \
					char* endptr;                                \
					ui_curr_cmd_param = strtol(argv[i],&endptr,10); \
					(SET_FUNCTION_NAME_X)(ui_curr_cmd_param);         \
					curr_cmd = COMMAND_UPDATE_COEFFS;            \
					print ("\n\r");                              \
				} else {                                         \
					print (STR_NEED_MORE_PARAMETERS);\
				}                                                \
			} else if (strcmp (argv[i], _SCMD_DIR_Y) == 0) {     \
				if (++i < argc) {                                \
					char* endptr;                                \
					ui_curr_cmd_param = strtol(argv[i],&endptr,10); \
					(SET_FUNCTION_NAME_Y)(ui_curr_cmd_param);         \
					curr_cmd = COMMAND_UPDATE_COEFFS;            \
					print ("\n\r");                              \
				} else {                                         \
					print (STR_NEED_MORE_PARAMETERS);\
				}                                                \
			} else {                                             \
				print (STR_WRONG_ARGUMENT);         \
			}                                                    \
		} else {                                                 \
			print (STR_NEED_MORE_PARAMETERS);        \
		}                                                        \
})
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
				print (STR_NEED_MORE_PARAMETERS);
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
				ui_curr_cmd_param = strtol(argv[i],&endptr,10);
				curr_cmd = COMMAND_WIDTH;
				print ("\n\r");
			} else {
				print (STR_NEED_MORE_PARAMETERS);
			}
		} else if (strcmp (argv[i], _CMD_SPEED) == 0) {
			if (++i < argc) {
				char* endptr;
				int16_t speed;
				speed = strtol(argv[i],&endptr,10);
				ui_curr_cmd_param = ((speed << 8) & 0xFF00) | (speed & 0xFF);
				if (++i < argc) {
					speed = strtol(argv[i],&endptr,10);
					ui_curr_cmd_param &= 0x00FF;
					ui_curr_cmd_param |= ((speed << 8) & 0xFF00);
				}
				curr_cmd = COMMAND_SPEED;
				print ("\n\r");
			} else {
				print (STR_NEED_MORE_PARAMETERS);
			}
		} else if (strcmp (argv[i], _CMD_STAB) == 0) {
			if (++i < argc) {
				if (strcmp (argv[i], _SCMD_SWITCH_ON) == 0) {
					curr_cmd = COMMAND_STAB_ON;
					print ("\n\r");
				} else if (strcmp (argv[i], _SCMD_SWITCH_OFF) == 0) {
					curr_cmd = COMMAND_STAB_OFF;
					print ("\n\r");
				} else {
					print ((char*)argv[i]);
					print (STR_WRONG_ARGUMENT);
				}
			} else {
				print (STR_NEED_MORE_PARAMETERS);
			}
		} else if (strcmp (argv[i], _CMD_ENC) == 0) {
			print_statistics();
			curr_cmd = COMMAND_GET_ENC;
//			PrintPidStat();
		} else if (strcmp (argv[i], _CMD_K) == 0) {
			PrintPidCoefs();
		} else if (strcmp (argv[i], _CMD_KP) == 0) {
			PARSE_PID_COEF(SetKp_X, SetKp_Y);
		} else if (strcmp (argv[i], _CMD_KI) == 0) {
			PARSE_PID_COEF(SetKi_X, SetKi_Y);
		} else if (strcmp (argv[i], _CMD_KD) == 0) {
			PARSE_PID_COEF(SetKd_X, SetKd_Y);
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
	} else if ((argc > 1) && (strcmp (argv[0], _CMD_STAB)==0)) { // if command needs subcommands
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

void TERM_RegisterPrintStatCallback(struct stat_module *obj)
{
	list_add(&obj->list, &stat_list);
}

void TERM_Task(void)
{
	eq_queue_param_u param;

	if (curr_cmd == COMMAND_EMPTY) goto exit;

	switch (curr_cmd){
	case COMMAND_WIDTH:
		param.uiParam = ui_curr_cmd_param;
		EQ_PutEventParam(CMD_WIDTH, param);
		break;
	case COMMAND_SPEED:
		param.uiParam = ui_curr_cmd_param;
		EQ_PutEventParam(CMD_SPEED, param);
		break;
	case COMMAND_STAB_ON:
		EQ_PutEvent(CMD_STAB_ON);
		break;
	case COMMAND_STAB_OFF:
		EQ_PutEvent(CMD_STAB_OFF);
		break;
	case COMMAND_GET_ENC:
		EQ_PutEvent(CMD_GET_ENC);
		break;
	default:
		break;
	}
	curr_cmd = COMMAND_EMPTY;

exit:
	return;
}


