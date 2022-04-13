#include "typedef.h"
#include <math.h>

#define BIT_SET(data, nbit)   ((data) |=  (1<<(nbit)))
#define BIT_CLEAR(data, nbit) ((data) &= ~(1<<(nbit)))
#define BIT_FLIP(data, nbit)  ((data) ^=  (1<<(nbit)))
#define BIT_CHECK(data, nbit) ((data) &   (1<<(nbit)))

#define GET_BIT(data, nbit) ({uint8_t retval; retval = ((data >> nbit) & 1); retval;})

#define PERCENT_100_2_DECIMAL ((uint16_t)10000)

//#define DECIMAL_RIGHT_SHIFT_U16(data, right_shift_decimal_places) ( (uint16_t)((data) *= pow(10,right_shift_decimal_places)) )

//#define DECIMAL_LEFT_SHIFT_U16(data, left_shift_decimal_places) ( (float)((data) *= 1/(float)pow(10,left_shift_decimal_places)) )

//#define CALCULATE_PERCENT(data, base) ( (float)data *= 100/((float)base) )

// Convert an float to uint16_t. Ex: 15.75 to 1575
#define DECIMAL_RIGHT_SHIFT_U16(data, right_shift_decimal_places) ({uint16_t retval; retval = (uint16_t)(data*pow(10,right_shift_decimal_places)); retval;})

// Convert an uint16_t to flaot, Ex: 1575 to 15.75
#define DECIMAL_LEFT_SHIFT_U16(data, left_shift_decimal_places) ({float retval; retval = data*1/ ((float)pow(10,left_shift_decimal_places)); retval;})

// Calcuate percentage = data/ base *100. Percentage will be in float. 
#define CALCULATE_PERCENT(data, base) ({float retval; retval = data*100/ ((float)base); retval;})

enum{
  BIT_LOW = (uint8_t)0,
  BIT_HI = (uint8_t)1
};

enum{
  ACTIVE_LOW = (uint8_t)0,
  ACTIVE_HI = (uint8_t)1 
};

enum{
  NORMAL_OPEN = (uint8_t)0,
  NORMAL_CLOSED = (uint8_t)1  
};

//enum{
//  ENABLE = (uint8_t)1,
//  DISABLE = (uint8_t)0  
//};