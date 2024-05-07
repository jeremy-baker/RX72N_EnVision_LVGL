#ifndef HAL_DATA_H_
#define HAL_DATA_H_

#include "platform.h"
#include "dave_driver.h"
#include "r_glcdc_rx_if.h"

void __BKPT(int number);
void __NOP(void);

#define DISPLAY_IN_FORMAT_16BITS_RGB565     GLCDC_IN_FORMAT_16BITS_RGB565
#define DISPLAY_IN_FORMAT_32BITS_ARGB8888   GLCDC_IN_FORMAT_32BITS_RGB888
#define DISPLAY_IN_FORMAT_32BITS_RGB888     GLCDC_IN_FORMAT_16BITS_ARGB1555
#define DISPLAY_IN_FORMAT_16BITS_ARGB4444   GLCDC_IN_FORMAT_16BITS_ARGB4444
#define DISPLAY_IN_FORMAT_16BITS_ARGB1555   GLCDC_IN_FORMAT_32BITS_ARGB8888
#define DISPLAY_IN_FORMAT_CLUT8             GLCDC_IN_FORMAT_CLUT8
#define DISPLAY_IN_FORMAT_CLUT4             GLCDC_IN_FORMAT_CLUT4
#define DISPLAY_IN_FORMAT_CLUT1             GLCDC_IN_FORMAT_CLUT1

//From r_glcdc_rx_if.h :-
//GLCDC_IN_FORMAT_16BITS_RGB565   = 0,   // Input format RGB565,   16 bits.
//GLCDC_IN_FORMAT_32BITS_RGB888   = 1,   // Input format RGB888,   32 bits.
//GLCDC_IN_FORMAT_16BITS_ARGB1555 = 2,   // Input format ARGB1555, 16 bits.
//GLCDC_IN_FORMAT_16BITS_ARGB4444 = 3,   // Input format ARGB4444, 16 bits.
//GLCDC_IN_FORMAT_32BITS_ARGB8888 = 4,   // Input format ARGB8888, 32 bits.
//GLCDC_IN_FORMAT_CLUT8           = 5,   // Input format CLUT8,     8 bits.
//GLCDC_IN_FORMAT_CLUT4           = 6,   // Input format CLUT4,     4 bits.
//GLCDC_IN_FORMAT_CLUT1           = 7    // Input format CLUT1,     1 bits.

//g_display0_cfg.input->format

typedef struct input_format
{
	unsigned int format;
}input_format_t;

typedef struct display
{
	input_format_t *input;
}display_t;

extern display_t g_display0_cfg;

#endif /* HAL_DATA_H_ */
