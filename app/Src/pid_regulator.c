#include "pid_regulator.h"

static unsigned encoder_x = 0;
static unsigned encoder_y = 0;
void SetEnc(unsigned x, unsigned y)
{
	encoder_x = x;
	encoder_y = y;
}

void GetEnc(unsigned *px, unsigned *py)
{
	*px = encoder_x;
	*py = encoder_y;
}

