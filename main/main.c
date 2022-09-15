#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_modbus_master.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <ctype.h>
#include "circularbuffer.h"
#include "esp_vfs_dev.h"

#define MB_PORT_NUM     2
#define MB_RTS			18
#define MB_RX			22
#define MB_TX			23
#define MB_SLAVE_ADDR   1
#define MB_DEV_SPEED    9600
#define MB_DEVICE_ADDR1	1
#define MB_DEVICE_ADDR2	2

#define PC_UART_PORT	1
#define PC_RX			4
#define PC_TX			5

#define LED_RED_PIN			33
#define LED_GREEN_PIN		32
#define LED_BLUE_PIN		27

#define MB_PAR_INFO_GET_TOUT   10 // Timeout for get parameter info

#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

#define k_CB_BUFFER_SIZE	50
#define k_HOLDING_REG_SIZE	10
#define k_CID_COUNT	128
#define k_CID_PARAM_KEY_SIZE	5
#define k_MODBUS_ERR_TIMEOUT	6 // 2 * 3 -> 3s
#define k_HALFWORD				0x1234
#define k_SEND_TO_PC			8 // 1.6 s
#define RESTART_CHAR			'\n'


#define CID_UPDATE() do{\
		sprintf(cid_param_key[i], "%d", i);\
		device_parameters[i].cid = i;\
		device_parameters[i].param_key = cid_param_key[i];\
		device_parameters[i].param_units= cid_param_key[i];\
		device_parameters[i].mb_slave_addr = i + 1;\
		device_parameters[i].mb_param_type = MB_PARAM_HOLDING;\
		device_parameters[i].mb_reg_start = 0;\
		device_parameters[i].mb_size = 2 * k_HOLDING_REG_SIZE;\
		device_parameters[i].param_offset = 0;\
		device_parameters[i].param_type = PARAM_TYPE_U32;\
		device_parameters[i].param_size = sizeof(uint32_t);\
		device_parameters[i].param_opts.opt1 = 0;\
		device_parameters[i].param_opts.opt2 = UINT32_MAX;\
		device_parameters[i].param_opts.opt3 = 1;\
		device_parameters[i].access = PAR_PERMS_READ_WRITE_TRIGGER;\
}while(0)\

typedef struct
{
	uint32_t bit0 :1U;
	uint32_t bit1 :1U;
	uint32_t bit2 :1U;
	uint32_t bit3 :1U;
	uint32_t bit4 :1U;
	uint32_t bit5 :1U;
	uint32_t bit6 :1U;
	uint32_t bit8 :1U;
} bits_t;

typedef struct
{
	// SOH 1
	uint32_t dw_id;
	uint32_t dw_ref;
	uint32_t dw_rec_id;
	uint16_t w_steps;
	uint8_t b_time_of_rest;
	uint8_t b_rest_stand_cnt;
}rec_data_t;

bits_t device_err;
uint32_t holding_regs[k_HOLDING_REG_SIZE];
uint32_t dw_rec_id = 0;
int32_t l_buff_state = 0;
uint16_t w_active_modbus_cnt = 0;
uint16_t w_halfword = 0;
char cid_param_key[k_CID_COUNT][k_CID_PARAM_KEY_SIZE];
bool ao_active_device[k_CID_COUNT];

TimerHandle_t timer_1sec;

mb_parameter_descriptor_t device_parameters[k_CID_COUNT];
uint16_t num_device_parameters = 0;

nvs_handle_t s_nvs;
CircularBufferContext cb;

rec_data_t s_rec_cb_buff[k_CB_BUFFER_SIZE];

void gpio_init_sequence(void);
void set_rgb(bool r, bool g, bool b);
void uart_init(void);
void master_init(void);
void master_op_task(void *arg);
void alarm_supervisor_task(void *arg);
void callback_timer_1sec(void *arg);
void set_modbus_parameters(void);
void create_timers(void);
void create_tasks(void);
void wait_for_updating_modbus_device_count(void);









void app_main(void)
{
	gpio_init_sequence();
	esp_vfs_dev_uart_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);
	nvs_flash_init();
	wait_for_updating_modbus_device_count();
	CircularBufferInit(&cb, s_rec_cb_buff, sizeof(s_rec_cb_buff), sizeof(s_rec_cb_buff[0]));
	set_modbus_parameters();
	master_init();
	create_timers();
	create_tasks();
}








#include "esp_console.h"

void gpio_init_sequence(void)
{
	gpio_reset_pin(LED_RED_PIN);
	gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
	gpio_reset_pin(LED_GREEN_PIN);
	gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(LED_BLUE_PIN);
	gpio_set_direction(LED_BLUE_PIN, GPIO_MODE_INPUT);
}

void set_rgb(bool r, bool g, bool b)
{
	gpio_set_level(LED_RED_PIN, r);
	gpio_set_level(LED_GREEN_PIN, g);
	gpio_set_level(LED_BLUE_PIN, b);
}

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(PC_UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PC_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PC_UART_PORT, PC_TX, PC_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_write_bytes(PC_UART_PORT, "1234\n", 5);

}

void master_init(void)
{
	// Initialize and start Modbus controller
	mb_communication_info_t comm =
	{
			.port = MB_PORT_NUM,
			.mode = MB_MODE_RTU,
			.baudrate = MB_DEV_SPEED,
			.parity = MB_PARITY_NONE };
	void *master_handler = NULL;

	esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
	err = mbc_master_setup((void*) &comm);
	// Set UART pin numbers
	err = uart_set_pin(MB_PORT_NUM, MB_TX, MB_RX, MB_RTS, UART_PIN_NO_CHANGE);
	err = mbc_master_start();
	err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);

	vTaskDelay(5);
	err = mbc_master_set_descriptor(
			&device_parameters[0],
			num_device_parameters);
	ESP_ERROR_CHECK(err);
}

void master_op_task(void *arg)
{
	uint8_t type = 0; // mb_descr_type_t, for uint32_t is 2
	static uint8_t i = 0, j = 0;
	esp_err_t err = ESP_OK;
	rec_data_t s_rec_data;
	const mb_parameter_descriptor_t *param_descriptor = NULL;

	TickType_t 	xLastWakeTime;
	TickType_t	wait =  pdMS_TO_TICKS(200);
	xLastWakeTime = xTaskGetTickCount();
	int digit;
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, wait);
		err = mbc_master_get_cid_info(i, &param_descriptor);
		if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL))
		{
			if(mbc_master_get_parameter(i,(char*) param_descriptor->param_key,(uint8_t*) holding_regs, &type) == ESP_OK)
			{
				ao_active_device[i] = true;
				if (param_descriptor->mb_param_type == MB_PARAM_HOLDING)
				{
					s_rec_data.dw_id = holding_regs[0];
					s_rec_data.dw_ref = holding_regs[1];
					s_rec_data.w_steps = holding_regs[2];
					s_rec_data.b_time_of_rest = holding_regs[3];
					s_rec_data.b_rest_stand_cnt = holding_regs[4];
					s_rec_data.dw_rec_id = holding_regs[5];

					if((s_rec_data.dw_id != 0) && (s_rec_data.dw_rec_id != 0))
					{l_buff_state = CircularBufferPushBack(&cb, (void*)&s_rec_data);}

									//				if (((holding_reg > param_descriptor->param_opts.max) || (holding_reg < param_descriptor->param_opts.min)))
									//				{
									//					//alarm state
									//				}
				}
			}
			else
			{
				ao_active_device[i] = false;
			}
		}

		if(++i >= w_active_modbus_cnt) { i = 0; }

		if((digit = getchar()) == RESTART_CHAR)
		{
			esp_restart();
		}

		if(++j >= k_SEND_TO_PC)
		{
			j = 0;
			if(!CircularBufferPopFront(&cb, &s_rec_data))
			{
				printf( "%s\n"
						"id: %d\n"
						"ref: %d\n"
						"step: %d\n"
						"time of rest: %d\n"
						"rest stand: %d\n"
						"rec ID: %d\n",
//						"a: %d\n",
						param_descriptor->param_key,
						s_rec_data.dw_id,
						s_rec_data.dw_ref,
						s_rec_data.w_steps,
						s_rec_data.b_time_of_rest,
						s_rec_data.b_rest_stand_cnt,
						s_rec_data.dw_rec_id
						);
			}
		}
	}
//	err = mbc_master_set_parameter(1, "param1", (uint8_t*)&holding_regs[2], &type);
//	if (err == ESP_OK)
//	{
//		ESP_LOGI("WRITE:","OK");
//	}
}

void alarm_supervisor_task(void *arg)
{
	uint32_t dw_device_cnt = 0;
	uint32_t dw_modbus_err_cnt = 0;
	TickType_t xLastWakeTime;
	TickType_t wait = pdMS_TO_TICKS(500);
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, wait);

		dw_device_cnt = 0;
		device_err.bit0 = false;
		device_err.bit1 = false;
		device_err.bit2 = false;

		// check modbus device states
		for(uint32_t i = 0; i < w_active_modbus_cnt; ++i)
		{
			 if(ao_active_device[i])
			 {
				 ++dw_device_cnt;
			 }
		}

		// handle the situation after the check which if at least one device or none of them doesn't work or "All is Well"
		if(dw_device_cnt == w_active_modbus_cnt)
		{
			dw_modbus_err_cnt = 0;
			set_rgb(1, 0, 1);
			if(!xTimerIsTimerActive(timer_1sec)) {xTimerStart(timer_1sec, 0);}
		}
		else if((dw_device_cnt != w_active_modbus_cnt) && (dw_device_cnt != 0))
		{
			if(++dw_modbus_err_cnt >= k_MODBUS_ERR_TIMEOUT)
			{
				set_rgb(0, 0, 1);
				if(!xTimerIsTimerActive(timer_1sec)) {xTimerStart(timer_1sec, 0);}
			}
		}

		else if(dw_device_cnt == 0)
		{
			if(++dw_modbus_err_cnt >= k_MODBUS_ERR_TIMEOUT)
			{
				set_rgb(0, 1, 1);
				if(!xTimerIsTimerActive(timer_1sec)) {xTimerStart(timer_1sec, 0);}
			}
		}
	}

}

void callback_timer_1sec(void *arg)
{
	set_rgb(1, 1, 1);
}

void set_modbus_parameters(void)
{
	num_device_parameters = w_active_modbus_cnt;
	for(uint16_t i = 0; i < w_active_modbus_cnt; ++i)
	{
		CID_UPDATE();
	}
}

void create_timers(void)
{
	timer_1sec = xTimerCreate
	                   ("timer_1sec",
	                     pdMS_TO_TICKS(1000),
	                     pdFALSE, // one-shot
	                     (void *)0,
						 callback_timer_1sec);
}

void create_tasks(void)
{
	xTaskCreatePinnedToCore(
			master_op_task,
			"master_op_task",
			4096,
			NULL,
			5,
			NULL,
			1);

	xTaskCreatePinnedToCore(
			alarm_supervisor_task,
			"alarm_supervisor_task",
			4096,
			NULL,
			5,
			NULL,
			1);
}

void wait_for_updating_modbus_device_count(void)
{
	enum{SIZE_OF_DIGIT = 4,
		WAITING_TIME =	700,
	};

	uint32_t dw_startup_waiting_time = 0;
	int digit = 0;
	char ac_arr[SIZE_OF_DIGIT] = {0};
	uint8_t i = 0;
	bool o_startup_lock = false;

	printf("Enter the receiver number: ");
	ac_arr[SIZE_OF_DIGIT - 1] = '\0';
	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(10));
		digit = getchar();

		if ((digit == '\n') || ((dw_startup_waiting_time > WAITING_TIME) && !o_startup_lock))
		{
			break;
		}

		if (isdigit(digit) && (i < SIZE_OF_DIGIT - 1))
		{
//			 printf("%c\n", digit); // purpose is to see on the Terminal
			ac_arr[i] = (char) digit;
			++i;
//			 dw_startup_waiting_time = 0;
			o_startup_lock = true;
			printf("%c\n", digit);
		}
		++dw_startup_waiting_time;
	}

	if (o_startup_lock)
	{
		sscanf(ac_arr, "%d", (int*) &w_active_modbus_cnt);
		printf("%d\n", w_active_modbus_cnt);
		nvs_open("storage", NVS_READWRITE, &s_nvs);
		nvs_set_u16(s_nvs, "halfword", k_HALFWORD);
		nvs_set_u16(s_nvs, "receiver_count", w_active_modbus_cnt);
		nvs_commit(s_nvs);
		nvs_close(s_nvs);
	}
	else
	{
		nvs_open("storage", NVS_READWRITE, &s_nvs);
		nvs_get_u16(s_nvs, "halfword", &w_halfword);
		nvs_get_u16(s_nvs, "receiver_count", &w_active_modbus_cnt);
		nvs_commit(s_nvs);
		nvs_close(s_nvs);
		printf("%d\n", w_active_modbus_cnt);
		if ((w_halfword != k_HALFWORD) || (w_active_modbus_cnt == 0))
		{
			for (;;)
			{
				vTaskDelay(pdMS_TO_TICKS(150));
				set_rgb(1, 1, 0);
				vTaskDelay(pdMS_TO_TICKS(150));
				set_rgb(1, 1, 1);
			}
		}
	}

}

