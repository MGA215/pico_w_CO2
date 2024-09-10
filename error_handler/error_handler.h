/**
 * @file error_handler.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Core 0 and 1 error handlers
 * @version 0.1
 * @date 2024-09-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __ERROR_HANDLER_H__
#define __ERROR_HANDLER_H__

#include "hardware/exception.h"

extern void error_handler_set_hardfault_core0(void);

extern void error_handler_set_hardfault_core1(void);

#endif