/*********************
 *      INCLUDES
 *********************/
#include "platform.h"
#include <stdbool.h>
#include "lv_port_disp.h"
#include "lvgl/src/display/lv_display_private.h"
#include "r_glcdc_rx_if.h"
#include "r_gpio_rx_if.h"
#include "r_glcdc_rx_pinset.h"

/*********************
 *      DEFINES
 *********************/


#define RGB_565_BLACK  (0)
#define RGB_565_RED    (0x1F << 11)
#define RGB_565_GREEN  (0x3F << 5)
#define RGB_565_BLUE   (0x1F << 0)


#define LCD_DISPON  GPIO_PORT_B_PIN_3
#define DISP_BLEN   GPIO_PORT_6_PIN_7

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void vsync_wait_cb(struct _lv_display_t * disp);


#define BYTES_PER_PIXEL 2 //LCD_CH0_IN_GR2_FORMAT = GLCDC_IN_FORMAT_16BITS_RGB565

static uint8_t g_framebuffer[2][LCD_CH0_IN_GR2_HSIZE * LCD_CH0_IN_GR2_VSIZE * BYTES_PER_PIXEL]__attribute__((section(".framebuffer"), aligned(64), used));

static SemaphoreHandle_t g_SemaphoreVsync = NULL;
static glcdc_cfg_t          g_config;
static glcdc_runtime_cfg_t  g_layer_change;


/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
	g_SemaphoreVsync = xSemaphoreCreateBinary();


    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    lv_display_t * disp = lv_display_create(LCD_CH0_IN_GR2_HSIZE, LCD_CH0_IN_GR2_VSIZE);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_flush_wait_cb(disp, vsync_wait_cb);
    lv_display_set_buffers(disp, &g_framebuffer[0][0], &g_framebuffer[1][0], sizeof(g_framebuffer[0]), LV_DISPLAY_RENDER_MODE_DIRECT);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    glcdc_err_t err;
    glcdc_runtime_cfg_t  layer_change;

    R_GPIO_PinControl(LCD_DISPON, GPIO_CMD_OUT_CMOS);
    R_GPIO_PinControl(DISP_BLEN, GPIO_CMD_OUT_CMOS);

    R_GPIO_PinWrite(DISP_BLEN, GPIO_LEVEL_LOW);

    /* Display OFF */
    R_GPIO_PinWrite(LCD_DISPON, GPIO_LEVEL_LOW);



    /* Fill the Frame buffer with a colour, to zero out info from previous execution runs */
    uint32_t count;
    uint16_t * p = (uint16_t *)&g_framebuffer[1][0];


    for (count = 0; count < sizeof(g_framebuffer)/2; count++)
    {
        *p++ = RGB_565_BLACK;
    }

    R_GLCDC_PinSet();

    err = R_GLCDC_Open(&g_config);
    if (GLCDC_SUCCESS != err)
    {
    	while(1);
    }

    err = R_GLCDC_Control(GLCDC_CMD_START_DISPLAY, &g_config);
    if (GLCDC_SUCCESS != err)
    {
    	while(1);
    }

    g_layer_change.input = g_config.input[GLCDC_FRAME_LAYER_2];
    g_layer_change.chromakey = g_config.chromakey[GLCDC_FRAME_LAYER_2];
    g_layer_change.blend = g_config.blend[GLCDC_FRAME_LAYER_2];

    layer_change.input.p_base = (uint32_t *)&g_framebuffer[1][0];

    do
    {
        err = R_GLCDC_LayerChange(GLCDC_FRAME_LAYER_2, &g_layer_change);
    } while (GLCDC_ERR_INVALID_UPDATE_TIMING == err);

    /* Display ON */
    R_GPIO_PinWrite(LCD_DISPON, GPIO_LEVEL_HIGH);

    /* Enable the backlight */
    R_GPIO_PinWrite(DISP_BLEN, GPIO_LEVEL_HIGH);
}

void glcdc_callback(glcdc_callback_args_t *p_args)
{

    if (GLCDC_EVENT_LINE_DETECTION == p_args->event)
    {
       BaseType_t context_switch;

       //
       // Set Vsync semaphore
       //
       xSemaphoreGiveFromISR(g_SemaphoreVsync, &context_switch);

       //
       // Return to the highest priority available task
       //
       portYIELD_FROM_ISR(context_switch);
    }
    else if (GLCDC_EVENT_GR1_UNDERFLOW == p_args->event)
    {
    	while(1);
    }
    else if (GLCDC_EVENT_GR2_UNDERFLOW == p_args->event)
    {
    	while(1);
    }
    else //DISPLAY_EVENT_FRAME_END
    {
    	while(1);
    }

}

static void vsync_wait_cb(lv_display_t * display)
{
    if(!lv_display_flush_is_last(display)) return;

    //
    // If Vsync semaphore has already been set, clear it then wait to avoid tearing
    //
    if (uxSemaphoreGetCount(g_SemaphoreVsync))
    {
        xSemaphoreTake(g_SemaphoreVsync, 10);
    }

    xSemaphoreTake(g_SemaphoreVsync, portMAX_DELAY);
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{

    FSP_PARAMETER_NOT_USED(area);
    glcdc_err_t err;
    //Display the frame buffer pointed by px_map

    if(!lv_display_flush_is_last(display)) return;

    g_layer_change.input.p_base = (uint32_t *)px_map;

    do
    {
        err = R_GLCDC_LayerChange(GLCDC_FRAME_LAYER_2, &g_layer_change);
    } while (GLCDC_ERR_INVALID_UPDATE_TIMING == err);

}
