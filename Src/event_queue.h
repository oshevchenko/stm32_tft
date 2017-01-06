#ifndef __EVENT_QUEUE_H__
#define __EVENT_QUEUE_H__

typedef enum QUEUE_EVENT_TAG {
	NO_EVENT = -1,
	TIMER1_EXPIRED,
	TIMER2_EXPIRED,
	CMD_WIDTH,
	CMD_OFFSET,
	CMD_HV_ON,
	CMD_HV_OFF
} eq_queue_event_e;

typedef union QUEUE_PARAM_TAG {
	uint32_t uiParam;
	int32_t iParam;
	void *pParam;
} eq_queue_param_u;

typedef struct QUEUE_ELEMENT_TAG {
	eq_queue_event_e event;
	eq_queue_param_u param;
} eq_queue_element_s;

typedef void (*eq_pErrorCallback)(void);

void EQ_PutEvent(eq_queue_event_e event);
void EQ_PutEventParam(eq_queue_event_e event, eq_queue_param_u param);
void EQ_GetEvent(eq_queue_element_s *ev);
void EQ_RegisterErrorCalback(eq_pErrorCallback err_callback);

#endif //#define __EVENT_QUEUE_H__
/******END OF FILE****/

