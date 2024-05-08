#include <string.h>
#include "FreeRTOS.h"
#include "platform.h"
#include "touch_ft5x06.h"
#include "r_gpio_rx_if.h"
#include "r_sci_iic_rx_if.h"
#include "r_irq_rx_if.h"
#include "r_mpc_rx_if.h"

static EventGroupHandle_t g_i2c_event_group;
SemaphoreHandle_t g_irq_binary_semaphore = NULL;

volatile sci_iic_info_t g_sci_iic_cfg;

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define FT5X06_DOWN          0
#define FT5X06_UP            1
#define FT5X06_CONTACT       2


#define FT5X06_SLAVE_ADDRESS 0x38

#define FT5X06_REG_TD_STATUS 0x02

#define extract_e(t) ((uint8_t) ((t).event))
#define extract_x(t) ((int16_t) (((t).x_msb << 8) | ((t).x_lsb)))
#define extract_y(t) ((int16_t) (((t).y_msb << 8) | ((t).y_lsb)))

/**********************************************************************************************************************
 * Type definitions
 **********************************************************************************************************************/
/* Driver-specific touch point register mapping */
typedef struct st_ft5x06_touch
{
    uint8_t  x_msb : 4;
    uint8_t        : 2;
    uint8_t  event : 2;
    uint8_t  x_lsb;

    uint8_t  y_msb : 4;
    uint8_t  id    : 4;
    uint8_t  y_lsb : 8;

    uint8_t  res1;
    uint8_t  res2;
} ft5x06_touch_t;

/* Complete FT5X06 data payload (number of active points + all five touch points) */ 
typedef struct st_ft5x06_payload
{
    uint8_t        num_points_active;
    ft5x06_touch_t data_raw[FT5X06_NUM_POINTS];
} ft5x06_payload_t;

#define I2C_TRANSFER_COMPLETE  (1<<0)
#define I2C_TRANSFER_ABORT     (1<<1)

#define I2C_TIMEOUT_MS         1000/portTICK_PERIOD_MS

#define TOUCH_SCI_IIC_CHANNEL   6

#define TOUCH_RESET    GPIO_PORT_6_PIN_6
#define TOUCH_IRQ      GPIO_PORT_3_PIN_4

#define TOUCH_SCL      GPIO_PORT_3_PIN_3
#define TOUCH_SDA      GPIO_PORT_3_PIN_2


/**********************************************************************************************************************
 * Function definitions
 **********************************************************************************************************************/
fsp_err_t i2c_wait(void);

void touch_irq_cb(void *pargs)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Set touch IRQ semaphore */
	/* Unblock the task by releasing the semaphore. */
	xSemaphoreGiveFromISR( g_irq_binary_semaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

/* Called from touch i2c isr routine */
void touch_i2c_callback(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t xResult = pdFAIL;

	/* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
	xHigherPriorityTaskWoken = pdFALSE;

	volatile sci_iic_return_t ret;
	sci_iic_mcu_status_t iic_status;
	sci_iic_info_t iic_info_ch;
	iic_info_ch.ch_no = TOUCH_SCI_IIC_CHANNEL;
	ret = R_SCI_IIC_GetStatus(&iic_info_ch, &iic_status);

    if (SCI_IIC_SUCCESS != ret)
    {
    	/* Call error processing for the R_SCI_IIC_GetStatus()function*/
    	while(1);
    }
    else
    {
		if (1 == iic_status.BIT.NACK)
		{
			/* Processing when a NACK is detected
			by verifying the iic_status flag. */
	        xResult = xEventGroupSetBitsFromISR(g_i2c_event_group, I2C_TRANSFER_ABORT, &xHigherPriorityTaskWoken );

		}
		else
		{
	        xResult = xEventGroupSetBitsFromISR(g_i2c_event_group, I2C_TRANSFER_COMPLETE, &xHigherPriorityTaskWoken );
		}
    }

    /* Was the message posted successfully? */
    if( pdFAIL != xResult)
    {
        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
        switch should be requested.  The macro used is port specific and will
        be either portYIELD_FROM_ISR() or portEND_SWITCHING_ISR() - refer to
        the documentation page for the port being used. */
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

/*******************************************************************************************************************//**
 * Basic function to wait for I2C comms completion
 **********************************************************************************************************************/

fsp_err_t i2c_wait(void)
{
    fsp_err_t ret = FSP_SUCCESS;
    EventBits_t uxBits;

    uxBits =  xEventGroupWaitBits(g_i2c_event_group,
                    I2C_TRANSFER_COMPLETE | I2C_TRANSFER_ABORT,
                    pdTRUE, //Clearbits before returning
                    pdFALSE, //either bit will do
                    I2C_TIMEOUT_MS  );

    if ((I2C_TRANSFER_COMPLETE & uxBits) == I2C_TRANSFER_COMPLETE)
    {
        ret = FSP_SUCCESS;
    }
    else if ((I2C_TRANSFER_ABORT & uxBits) == I2C_TRANSFER_ABORT)
    {
        ret = FSP_ERR_ABORTED;
    }
    else
    {
        /* xEventGroupWaitBits() returned because of timeout */
        ret = FSP_ERR_TIMEOUT;
    }

    return ret;
}

/*******************************************************************************************************************//**
 * Reset the FT5X06
 **********************************************************************************************************************/
static void ft5x06_reset ()
{
    R_GPIO_PinControl(TOUCH_RESET, GPIO_CMD_OUT_CMOS);
    R_GPIO_PortDirectionSet(GPIO_PORT_6, GPIO_DIRECTION_OUTPUT, (1<<6));

    /** Reset touch chip by setting GPIO reset pin low. */
    R_GPIO_PinWrite(TOUCH_RESET, GPIO_LEVEL_LOW);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_MILLISECS);

    /** Release touch chip from reset */
    R_GPIO_PinWrite(TOUCH_RESET, GPIO_LEVEL_HIGH);

    /** Wait 10 ms. */
    R_BSP_SoftwareDelay(10, BSP_DELAY_MILLISECS);
}



/*******************************************************************************************************************//**
 * Initialize the connection with the FT5X06 touch controller
 *
 * @param      p_i2c_instance  I2C Master instance to use for communication
 * @param      i2c_semaphore   Semaphore indicating I2C completion
 * @param[in]  reset_pin       Pin connected to FT5X06 reset line
 **********************************************************************************************************************/
fsp_err_t ft5x06_init(void)
{

	sci_iic_return_t err;
    irq_err_t result;
    mpc_config_t config;
    irq_handle_t my_handle;

	g_i2c_event_group = xEventGroupCreate();
    if(NULL == g_i2c_event_group )
    {
        while(1);
    }

	g_irq_binary_semaphore = xSemaphoreCreateBinary();
    if(NULL == g_irq_binary_semaphore )
    {
        while(1);
    }


    /* Reset FT5X06 controller */
    ft5x06_reset();

    g_sci_iic_cfg.dev_sts      = SCI_IIC_NO_INIT;
    g_sci_iic_cfg.ch_no        = TOUCH_SCI_IIC_CHANNEL;

    /* Open I2C peripheral */
    err = R_SCI_IIC_Open((sci_iic_info_t *)&g_sci_iic_cfg);
    if (SCI_IIC_SUCCESS != err)
    {
    	while(1);
    }

    result = R_IRQ_Open(IRQ_NUM_4, IRQ_TRIG_FALLING, IRQ_PRI_5, &my_handle, touch_irq_cb);
    if (IRQ_SUCCESS != result)
    {
    	while(1);
    }

    /* Set P34 to be used as IRQ pin */
    config.analog_enable = false;
    config.irq_enable = true;
    config.pin_function = 0;
    result= R_MPC_Write(TOUCH_IRQ, &config);
	if (MPC_SUCCESS != result)
	{
		while(1);
	}

    /* Enable interrupt */
    result = R_IRQ_InterruptEnable (my_handle, true);
    if (FSP_SUCCESS != result)
    {
    	while(1);
    }

    return err;
}

/*******************************************************************************************************************//**
 * Get all touch data from the FT5X06 touch controller
 * @param      touch_data      Pointer to struct for output touch data
 **********************************************************************************************************************/
void ft5x06_payload_get (touch_data_t * touch_data)
{
    touch_coord_t    new_touch;
    ft5x06_payload_t touch_payload;
    fsp_err_t err;
    uint8_t iic_write_data;
    uint8_t slave_addr_ft5x06;

    /* Clear payload struct */
    memset(&touch_payload, 0, sizeof(ft5x06_payload_t));

    /* Read the data about the touch point(s) */
    slave_addr_ft5x06 = FT5X06_SLAVE_ADDRESS;
    iic_write_data    = FT5X06_REG_TD_STATUS;

    g_sci_iic_cfg.p_slv_adr   = &slave_addr_ft5x06;
    g_sci_iic_cfg.callbackfunc = &touch_i2c_callback;
    g_sci_iic_cfg.p_data1st    = &iic_write_data;
    g_sci_iic_cfg.p_data2nd    = (uint8_t *)&touch_payload;
    g_sci_iic_cfg.cnt1st       = sizeof(iic_write_data);
    g_sci_iic_cfg.cnt2nd       = sizeof(ft5x06_payload_t);
    g_sci_iic_cfg.dev_sts      = SCI_IIC_NO_INIT;
    g_sci_iic_cfg.ch_no        = TOUCH_SCI_IIC_CHANNEL;

    /* Read TD_STATUS through all five TOUCHn_** register sets */
    err = R_SCI_IIC_MasterReceive((sci_iic_info_t *)&g_sci_iic_cfg);
    if (SCI_IIC_SUCCESS != err)
    {
        while(1);
    }
    else
    {
    	while((SCI_IIC_FINISH != g_sci_iic_cfg.dev_sts) && (SCI_IIC_NACK != g_sci_iic_cfg.dev_sts));
    	if (SCI_IIC_NACK == g_sci_iic_cfg.dev_sts)
    	{
    		return;
    	}
    }

    /* Process the raw data for the touch point(s) into useful data */
    for(uint8_t i = 0; i < FT5X06_NUM_POINTS; i++)
    {
        new_touch.x     = (uint16_t) extract_x(touch_payload.data_raw[i]);
        new_touch.y     = (uint16_t) extract_y(touch_payload.data_raw[i]);
        new_touch.event = extract_e(touch_payload.data_raw[i]);

        /* Set event type based on received data */
        switch(new_touch.event)
        {
            case FT5X06_DOWN:
                touch_data->point[i].event = TOUCH_EVENT_DOWN;
                break;
            case FT5X06_UP:
                touch_data->point[i].event = TOUCH_EVENT_UP;
                break;
            case FT5X06_CONTACT:
                /* Check if the point is moving or not */
                if ((touch_data->point[i].x != new_touch.x) || (touch_data->point[i].y != new_touch.y))
                {
                    touch_data->point[i].event = TOUCH_EVENT_MOVE;
                }
                else
                {
                    touch_data->point[i].event = TOUCH_EVENT_HOLD;
                }
                break;
            default:
                touch_data->point[i].event = TOUCH_EVENT_NONE;
                break;
        }

        /* Set new coordinates */
        touch_data->point[i].x = new_touch.x;
        touch_data->point[i].y = new_touch.y;
    }

    /* Pass the number of active touch points through */
    touch_data->num_points = touch_payload.num_points_active;
}
