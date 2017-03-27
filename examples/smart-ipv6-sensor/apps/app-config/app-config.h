/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \addtogroup apps
 * @{
 *
 * \defgroup app-config Application configuration storage
 * @{
 *
 * \file
 *  Header file for the smart settings library
 *
 * \author
 * Darko Petrovic
 *
 */
#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#include "contiki-conf.h"
#include "cfs-coffee-arch.h"

#include "lib/list.h"

#ifndef SETTINGS_CONF_NB_PARAM
#define SETTINGS_NB_PARAM     10
#else
#define SETTINGS_NB_PARAM     SETTINGS_CONF_NB_PARAM
#endif

#define APP_CONFIG_MAX_FILEPATH_LEN       COFFEE_NAME_LENGTH
#define MAX_PARAM_NAME_LEN                40
#define MAX_PARAM_VALUE_LEN               40
#define MAX_PARAM_LIST                    10

#define APP_CONFIG_HOME           "cfg"
#define APP_CONFIG_GENERAL        "main"
#define APP_CONFIG_RESOURCES      "res"
#define APP_CONFIG_USER           "user"

#ifndef APP_CONFIG_CONF_STORAGE_EEPROM
#define APP_CONFIG_STORAGE_EEPROM     0
#else
#define APP_CONFIG_STORAGE_EEPROM     APP_CONFIG_CONF_STORAGE_EEPROM
#endif

#ifndef APP_CONFIG_CONF_STORAGE_COFFEE
#define APP_CONFIG_STORAGE_COFFEE     0
#else
#define APP_CONFIG_STORAGE_COFFEE     APP_CONFIG_CONF_STORAGE_COFFEE
#endif

enum {
  FILENAME_ONLY,
  FULL_PATH
};


struct parameter;

struct parameter {
  struct parameter *next;
  const char* context;
  const char* name;
  const char* default_value;
  int32_t value;
  uint8_t is_string;
  uint8_t (*callback)(struct parameter *s);
};

/*struct parameter {
  const char* name;
  const char* value;
};*/

void app_config_init(void);
uint8_t app_config_create_parameter(const char* folder, const char* name, const char* default_value, void* callback);
struct parameter* app_config_parameter_lookup(const char* context, const char* name);
void* app_config_get_parameter_value(const char* context, const char* name);
int32_t app_config_edit_parameter(const char* context, const char* name, const char* strvalue, int32_t intvalue);
struct parameter* app_config_parameters_list_head(void);

#endif /* _APP_CONFIG_H */

/** @} */
/** @} */
