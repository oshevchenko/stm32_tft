#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "stm32_microrl_misc.h"
#include "list.h"
#include "logger.h"

static struct list_head list_array_log_data[LOGGER_ARRAY_SIZE];
static int m_current = 0;
static LIST_HEAD(logger_module_list);

void LOGGER_Trigger()
{
	int i;
	m_current = 0;
	for (i = 0; i < LOGGER_ARRAY_SIZE; i++) {
		INIT_LIST_HEAD(&list_array_log_data[i]);
	}
}

void LOGGER_RegisterModule(st_logger_module *obj)
{
	list_add_tail(&obj->list, &logger_module_list);
}

void LOGGER_GetData()
{
	st_logger_module *j = NULL;
	st_logger_data *obj = NULL;

	if (m_current >= LOGGER_ARRAY_SIZE) goto exit;
	list_for_each_entry(j, &logger_module_list, list) {
		obj = j->getLogDataCallback();
		list_add_tail(&obj->list, &list_array_log_data[m_current]);
	}
	m_current++;

exit:
	return;
}
static int m_i = LOGGER_ARRAY_SIZE;
void LOGGER_StartStat()
{
	m_i = 0;
}
void LOGGER_PrintData()
{
	char str_mod[LOGGER_MAX_STR_LEN] = {'\0',};
	char str[LOGGER_MAX_STR_LEN] = {'\0',};
	st_logger_data *j = NULL;
	int k;
	for (k=0; k < 5; k++) {
		if (m_i >= LOGGER_ARRAY_SIZE) goto exit;
		str[0] = '\0';
		snprintf(str, LOGGER_MAX_STR_LEN, "str:%d:",m_i);
		list_for_each_entry(j, &list_array_log_data[m_i], list) {
			str_mod[0] = '\0';
			j->sprintfDataCallback(str_mod, j->p_data, LOGGER_MAX_STR_LEN);
			strncat(str, str_mod, LOGGER_MAX_STR_LEN);
		}
		print(str);
		print ("\n\r");
		m_i++;
	}
exit:
	return;
}
