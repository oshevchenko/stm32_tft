/**
  ******************************************************************************
  * File Name          : event_queue.c
  * Description        : Simple events queue.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include "event_queue.h"

#define EVENT_QUEUE_SIZE 32
static eq_pErrorCallback ErrorCallback = NULL;
static eq_queue_element_s queue[EVENT_QUEUE_SIZE];
static int pAdd = 0;
static int pGet = 0;


void EQ_RegisterErrorCalback(eq_pErrorCallback err_callback)
{
	ErrorCallback = err_callback;
}

void EQ_PutEvent(eq_queue_event_e event)
{
	if (((pAdd + 1) & (EVENT_QUEUE_SIZE - 1)) == pGet) {
		// Queue is full! Call the Error callback!
		if (ErrorCallback) ErrorCallback();
		return;
	}
	memset(&queue[pAdd], 0, sizeof(eq_queue_element_s));
	queue[pAdd].event = event;
	pAdd = (pAdd + 1) & (EVENT_QUEUE_SIZE - 1);
}

void EQ_PutEventParam(eq_queue_event_e event, eq_queue_param_u param)
{
	if (((pAdd + 1) & (EVENT_QUEUE_SIZE - 1)) == pGet) {
		// Queue is full! Call the Error callback!
		if (ErrorCallback) ErrorCallback();
		return;
	}
	queue[pAdd].event = event;
	queue[pAdd].param = param;
	pAdd = (pAdd + 1) & (EVENT_QUEUE_SIZE - 1);
}

void EQ_GetEvent(eq_queue_element_s *ev)
{
	if (pAdd == pGet) {
		memset(ev, 0, sizeof(eq_queue_element_s));
		ev->event = NO_EVENT;
	} else {
		ev->event = queue[pGet].event;
		ev->param = queue[pGet].param;
		pGet = (pGet + 1) & (EVENT_QUEUE_SIZE - 1);
	}
}

/******END OF FILE****/

