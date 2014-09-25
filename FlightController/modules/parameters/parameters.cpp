#include "stm32f4xx_conf.h"
#include "parameters.h"
#include <cstring>

////////////////////////////////////////// END OF HEADER ////////////


//These are set by the Linker
extern struct param_s _param_start;
extern struct param_s _param_stop;

namespace
{

static struct param_s * params; // Pointer to first Parameter.
static int paramsLen; // Count of all entries in params sector
static int paramsCount = 0; // Cout of params.

}

void InitParams()
{
	params = &_param_start;
	paramsLen = &_param_stop - &_param_start;

	for (int i=0; i<paramsLen; i++)
	{
		if(!(params[i].type & PARAM_GROUP))
			paramsCount++;
	}
}

uint16_t GetParamsCount()
{
	return paramsCount;
}

bool GetParam(param_s& paramDesk, uint16_t paramIndex)
{
	int parC = 0;
	for (int i=0; i<paramsLen; i++)
	{
		param_s p = params[i];
		if(!(p.type & PARAM_GROUP))
		{
			if (parC == paramIndex)
			{
				paramDesk = p;
				return true;
			}
			parC++;
		}
	}
	return false;
}

int16_t GetParam(param_s& paramDesk, const char* paramName)
{
	int parC = 0;
	for (int i=0; i<paramsLen; i++)
	{
		param_s p = params[i];
		if(!(p.type & PARAM_GROUP))
		{
			if (strcmp(p.name, paramName) == 0)
			{
				paramDesk = p;
				return parC;
			}
			parC++;
		}
	}
	return -1;
}

//// TESTING PURPOSES

float beta = 2;
float teta = 2.3;
__attribute__((section(".persist"))) float dzeta = 3.14;

PARAM_GROUP_START(TEST_PARAMS)
	PARAM_ADD(PARAM_FLOAT, TEST_BETA, &beta)
	PARAM_ADD(PARAM_FLOAT, TEST_TETA, &teta)
	PARAM_ADD(PARAM_FLOAT, TEST_ZETA, &dzeta)
PARAM_GROUP_STOP(TEST_PARAMS)
