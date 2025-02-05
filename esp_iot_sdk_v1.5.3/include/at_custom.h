
/*
 * custom_at.h
 *
 * This file is part of Espressif's AT+ command set program.
 * Copyright (C) 2013 - 2016, Espressif Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 3 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CUSTOM_AT_H_
#define CUSTOM_AT_H_

#include "c_types.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
  char *at_cmdName;
  int8_t at_cmdLen;
  void (*at_testCmd)(uint8_t id);
  void (*at_queryCmd)(uint8_t id);
  void (*at_setupCmd)(uint8_t id, char *pPara);
  void (*at_exeCmd)(uint8_t id);
}at_funcationType;

typedef void (*at_custom_uart_rx_intr)(uint8* data,int32 len);

typedef void (*at_custom_response_func_type)(const char *str);

typedef void (*at_fake_uart_tx_func_type)(const uint8*data,uint32 length);

extern uint8 at_customLinkMax;

/**
  * @brief  Response "OK" to uart.
  * @param  None
  * @retval None
  */
void at_response_ok(void);
/**
  * @brief  Response "ERROR" to uart.
  * @param  None
  * @retval None
  */
void at_response_error(void);
/**
  * @brief  Response string.
  *  It is equivalent to at_port_print,if not call at_register_response_func or call at_register_response_func(NULL);
  *  It will run custom response function,if call at_register_response_func and parameter is not NULL.
  * @param  string
  * @retval None
  */
void at_response(const char *str);
/**
  * @brief  register custom response function.
  * @param  response_func: the function that will run when call at_response
  * @retval None
  */
void at_register_response_func(at_custom_response_func_type response_func);
/**
  * @brief  Task of process command or txdata.
  * @param  custom_at_cmd_array: the array of at cmd that custom defined
  *         cmd_num : the num of at cmd that custom defined
  * @retval None
  */
void at_cmd_array_regist(at_funcationType *custom_at_cmd_array,uint32 cmd_num);
/**
  * @brief  get digit form at cmd line.the maybe alter pSrc
  * @param  p_src: at cmd line string
  *         result:the buffer to be placed result
  *         err : err num
  * @retval TRUE:
  *         FALSE:
  */
bool at_get_next_int_dec(char **p_src,int*result,int* err);
/**
  * @brief  get string form at cmd line.the maybe alter pSrc
  * @param  p_dest: the buffer to be placed result
  *         p_src: at cmd line string
  *         max_len :max len of string excepted to get
  * @retval None
  */
int32 at_data_str_copy(char *p_dest, char **p_src, int32 max_len);

/**
  * @brief  initialize at module
  * @param  None
  * @retval None
  */
void at_init(void);
/**
  * @brief  print string to at port
  * @param  string
  * @retval None
  */
void at_port_print(const char *str);
/**
  * @brief  print custom information when AT+GMR
  * @param  string
  * @retval None
  */
void at_set_custom_info(char* info);
/**
  * @brief  if current at command is processing,you can call at_enter_special_state,
  *         then if other comamnd coming,it will return busy.
  * @param  None
  * @retval None
  */
void at_enter_special_state(void);
/**
  * @brief  
  * @param  None
  * @retval None
  */
void at_leave_special_state(void);
/**
  * @brief  get at version
  * @param  None
  * @retval at version
  *         bit24~31: at main version
  *         bit23~16: at sub version
  *         bit15~8 : at test version
  *         bit7~0  : customized version
  */
uint32 at_get_version(void);

/**
  * @brief  register custom uart rx interrupt function
  * @param  rx_func: custom uart rx interrupt function.
  * If rx_func is non-void,when rx interrupt comming,it will call rx_func(data,len),
  * data is the buffer of data,len is the length of data.Otherwise,it will run AT rx function.
  * @retval None
  */
void at_register_uart_rx_intr(at_custom_uart_rx_intr rx_func);
/**
  * @brief notify at module that has receive data
  * @param  data: data buffer.
  * @param  length: data length
  * @retval data len,if ok len == length
  */
uint32 at_fake_uart_rx(uint8* data,uint32 length);

/**
  * @brief enable fake uart,and register fake uart tx
  * @param  enable: enable fake uart.
  * @param  at_fake_uart_tx_func: 
  * @retval data len,if ok len == length
  */
bool at_fake_uart_enable(bool enable,at_fake_uart_tx_func_type at_fake_uart_tx_func);

/**
  * @brief set at escape character
  * @param  ch: escape character.
  * @retval TRUE,if set ok,otherwize FALSE.
  */
bool at_set_escape_character(uint8 ch);


#ifdef __cplusplus
}
#endif


#endif
