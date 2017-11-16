#ifndef __LOGGER_H__
#define __LOGGER_H__
#include "list.h"
#define LOGGER_MAX_STR_LEN 100
#define LOGGER_ARRAY_SIZE 100

typedef struct logger_data
{
	void *p_data;
    int (*sprintfDataCallback)(char*, void*, int);
	struct list_head list;
} st_logger_data;

typedef struct logger_module
{
	struct list_head list;
	st_logger_data* (*getLogDataCallback)();
} st_logger_module;

void LOGGER_Trigger();
void LOGGER_Next();
void LOGGER_RegisterModule(st_logger_module *obj);
void LOGGER_PrintData();
void LOGGER_GetData();
void LOGGER_Task();
void LOGGER_StartStat();
void LOGGER_PrintStatistics();
typedef struct stat_module
{
        struct list_head list;
        void (*printStatFunc)(char*, int);
} st_stat_module;

void LOGGER_RegisterPrintStatCallback(struct stat_module *obj);

#endif
