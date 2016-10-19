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
 * \addtogroup app-config
 * @{
 *
 * \file
 *
 *
 *  \author
 *  Darko Petrovic
 */
#include "contiki.h"
#include "app-config.h"
#include "cfs.h"

#include "lib/list.h"
#include "lib/memb.h"

#include <stdlib.h> /* strtol */

#define MAYBE_SOMEDAY     0

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

LIST(parameters_list);
MEMB(parameters_mem, struct parameter, MAX_PARAM_LIST);

static uint8_t settings_count;

void
app_config_init(void)
{
  list_init(parameters_list);
  memb_init(&parameters_mem);

  /* The problem is when we update a parameter with a value longer that the old one,
   * we need to move someway the following text otherwise it is overwritten. */
#if MAYBE_SOMEDAY
  fd = cfs_open(APP_CONFIG_GENERAL, CFS_WRITE);

  txtbuf =  "router=1\n"
            "rdc_enable_period=30\n"
            "alive_message_period=310\n"
            "bripaddr=aaaa::212:4b00:40e:fadb\n"
            "energest_enable=1250\n"
            "aro-registration=10\n"
      ;

  r = cfs_write(fd, txtbuf, strlen(txtbuf));

  cfs_close(fd);
#endif
}
#if APP_CONFIG_STORAGE_COFFEE
static int8_t
file_exist(const char* filepath)
{
  static struct cfs_dir dir;
  struct cfs_dirent dirent;
  uint8_t file_exist=0;
  cfs_opendir(&dir, 0);
  while(cfs_readdir(&dir, &dirent) == 0)
  {
    if(!strncmp(dirent.name, filepath, strlen(filepath))){
      file_exist=1;
      break;
    }
  }
  cfs_closedir(&dir);
  return file_exist;
}
#endif /* APP_CONFIG_STORAGE_COFFEE */

uint8_t
app_config_create_parameter(const char* context, const char* name, const char* default_value, void* callback)
{
#if APP_CONFIG_STORAGE_COFFEE
  static int fd = 0;
  char filepath[APP_CONFIG_MAX_FILEPATH_LEN];
  int r, len;
  char buf[MAX_PARAM_VALUE_LEN+1];
#endif /* APP_CONFIG_STORAGE_COFFEE */

  struct parameter *p;
  uint32_t value;

  /* Avoid inserting duplicate entries. */
  p = app_config_parameter_lookup(context, name);
  if(p != NULL) {
    PRINTF("APPCFG: (warning) Parameter already exist.\n");
    return 1;
  }

  /* Allocate a new entry. */
  p = memb_alloc(&parameters_mem);
  if(p == NULL) {
    PRINTF("APPCFG: (error) No more space for a new parameter.\n");
    return 1;
  }

  p->context = context;
  p->name = name;
  p->default_value = default_value;
  /* If default_value is numeric we set the value directly, otherwise the string
   * value is stored in memory and retrieved from there.  */
  if((value=strtol(default_value, NULL, 10)) != 0 || !strncmp(default_value, "0", 1)){
    p->value = value;
    p->is_string = 0;
  } else {
    p->value = 0;
    p->is_string = 1;
  }
  p->callback = callback;

#if APP_CONFIG_STORAGE_COFFEE
  sprintf(filepath, "%s/%s/%s", APP_CONFIG_HOME, context, name);
  memset(buf, 0, sizeof(buf));
  /* If file exists we read the value and update the parameter. */
  if(file_exist(filepath)){
    fd = cfs_open(filepath, CFS_READ);
    cfs_seek(fd, 0, CFS_SEEK_SET);
    len = cfs_read(fd, buf, MAX_PARAM_VALUE_LEN);
    cfs_close(fd);

    if(len > 0) {
      // update the parameter value with stored value
      if((value=strtol(buf, NULL, 10)) != 0 || !strncmp(buf, "0", 1)){
        p->value = value;
        p->is_string = 0;
        PRINTF("APPCFG: Parameter updated with value from memory: %lu.\n", p->value);
      } else {
        p->value = 0;
        p->is_string = 1;
        PRINTF("APPCFG: Parameter updated with value from memory: '%s'.\n", buf);
      }
    } else {
      PRINTF("APPCFG: (error) Couldn't read parameter value.\n");
      return 1;
    }

  } else {
    fd = cfs_open(filepath, CFS_WRITE);
    r = cfs_write(fd, default_value, strlen(default_value));
    cfs_close(fd);
    if(r!=strlen(default_value)){
      memb_free(&parameters_mem, p);
      PRINTF("APPCFG: (error) Couldn't store the parameter correctly.\n");
      return 1;
    }
  }

#endif /* APP_CONFIG_STORAGE_COFFEE */

  list_add(parameters_list, p);
  settings_count++;

  PRINTF("APPCFG: Create parameter (%d) '%s'=", settings_count, p->name);

#if DEBUG
#if APP_CONFIG_STORAGE_COFFEE
  if(p->is_string){
    if(buf[0]){
      PRINTF("'%s' ", buf); // the value from memory
    } else {
      PRINTF("'%s'* ", p->default_value);
    }
  } else {
    PRINTF("%lu ", p->value);
  }
#else /* APP_CONFIG_STORAGE_COFFEE */
  PRINTF("%lu ", p->value);
#endif
#endif /* DEBUG */

#if APP_CONFIG_STORAGE_COFFEE
  PRINTF("stored in '%s'", filepath);
#endif

  PRINTF(".\n");

  if (p->callback){
    p->callback(p);
  }

  return 0;

}

struct parameter*
app_config_parameter_lookup(const char* context, const char* name)
{
  struct parameter *p;
  for(p = list_head(parameters_list); p != NULL; p = list_item_next(p)) {
    if( !strcmp (context, p->context) && !strcmp (name, p->name) )
    {
      return p;
    }
  }
  return NULL;
}

void*
app_config_get_parameter_value(const char* context, const char* name)
{

#if APP_CONFIG_STORAGE_COFFEE
  static int fd = 0;
  int len;
  char filepath[APP_CONFIG_MAX_FILEPATH_LEN];
  char buf[MAX_PARAM_VALUE_LEN];
  static char param_value[MAX_PARAM_VALUE_LEN+1];
#endif /* APP_CONFIG_STORAGE_COFFEE */

  struct parameter *p;

  p = app_config_parameter_lookup(context, name);
  if( p == NULL ){
    PRINTF("APPCFG: (warning) Parameter doesn't exist.\n");
    return NULL;
  }

  if(!p->is_string){
    return &p->value;
  }
#if APP_CONFIG_STORAGE_COFFEE
  else {

    sprintf(filepath, "%s/%s/%s", APP_CONFIG_HOME, context, name);
    fd = cfs_open(filepath, CFS_READ);

    cfs_seek(fd, 0, CFS_SEEK_SET);
    len = cfs_read(fd, buf, MAX_PARAM_VALUE_LEN);

    if(len > 0) {
      strncpy(param_value, buf, len);
      param_value[len] = '\0';
      cfs_close(fd);
      return param_value;
    } else {
      cfs_close(fd);
      return NULL;
    }
  }
#endif /* APP_CONFIG_STORAGE_COFFEE */
  return NULL;
}

int32_t
app_config_edit_parameter(const char* context, const char* name, const char* strvalue, uint32_t intvalue)
{
#if APP_CONFIG_STORAGE_COFFEE
  static int fd = 0;
  char filepath[APP_CONFIG_MAX_FILEPATH_LEN];
  int r, len;
#endif /* APP_CONFIG_STORAGE_COFFEE */

  char buf[MAX_PARAM_VALUE_LEN];
  struct parameter *p;
  uint8_t error = 0;
  static uint32_t current_value;

  p = app_config_parameter_lookup(context, name);
  if( p != NULL ){

    current_value = p->value;

    if(strvalue != NULL){
      /* The parameter value become a string. */
      p->is_string = 1;
      p->value = 0;
    } else {
      /* The parameter value become an integer. */
      p->is_string = 0;
      p->value = intvalue;
    }

    /* Before storing the parameter in flash, verify that the callback works
     * with the new value. The callback may have updated the parameter value.
     */
    if (p->callback){
      error = p->callback(p);
    }

    if(!error){

#if APP_CONFIG_STORAGE_COFFEE
      sprintf(filepath, "%s/%s/%s", APP_CONFIG_HOME, context, name);
      /* The parameter must exist to edit it. */
      if( !file_exist(filepath) ){
        PRINTF("APPCFG: Parameter doesn't exist!'\n");
        return 1;
      }

      /* Get the old parameter value length. */
      fd = cfs_open(filepath, CFS_READ);
      cfs_seek(fd, 0, CFS_SEEK_SET);
      len = cfs_read(fd, buf, MAX_PARAM_VALUE_LEN);
      cfs_close(fd);

      /* Clear the content by removing the file if the value is
       * longer that the new value. */
      if(p->is_string){
        strcpy(buf, strvalue);
      } else {
        sprintf(buf, "%lu", intvalue);
      }
      if(len>strlen(buf)){
        cfs_remove(filepath);
      }

      fd = cfs_open(filepath, CFS_WRITE);
      cfs_seek(fd, 0, CFS_SEEK_SET);
      r = cfs_write(fd, buf, strlen(buf));
      cfs_close(fd);

      if(r!=strlen(buf)){
        error = 1;
      }
#else /* APP_CONFIG_STORAGE_COFFEE */
      if(p->is_string){
        strcpy(buf, strvalue);
      } else {
        sprintf(buf, "%lu", intvalue);
      }
#endif /* APP_CONFIG_STORAGE_COFFEE */

    } else {
      // restore back the previous value
      p->value = current_value;
    }
  } else {
    // parameter doesn't exist
    error = 1;
  }

  if(!error){
    PRINTF("APPCFG: Update parameter %s=%s in '%s'.\n", p->name, buf, p->context);
  } else {
    PRINTF("APPCFG: (error) Fail to update parameter %s = (%s|%lu) in '%s'.\n", p->name, strvalue, intvalue, p->context);
  }

  return error;
}

struct parameter*
app_config_parameters_list_head(void)
{
  return list_head(parameters_list);
}

#if 0
struct parameter*
app_config_get_list_params(const char* folderpath, uint8_t full_path)
{
  static struct cfs_dir dir;
  struct cfs_dirent dirent;
  cfs_opendir(&dir, 0);
  char path[APP_CONFIG_MAX_FILEPATH_LEN];
  char* param_name_p;
  char* param_value_p;
  uint8_t i=0;

  sprintf(path, "config/%s", folderpath);

  //memset(list_parameters, 0, sizeof(struct parameter)*MAX_PARAM_LIST);

  while(cfs_readdir(&dir, &dirent) == 0)
  {
    if(!strncmp(dirent.name, path, strlen(path))){
      if(full_path == FULL_PATH){
        // get the full path of the parameter
        param_name_p = dirent.name;
        //param_value_p = app_config_get_parameter(strchr(dirent.name, '/')+1);
      } else {
        // get only the parameteter's name
        param_name_p = strrchr(dirent.name, '/')+1;
        //param_value_p = app_config_get_parameter(strchr(dirent.name, '/')+1);
      }

      //strncpy(list_parameters[i].name, param_name_p, strlen(param_name_p));
      //strncpy(list_parameters[i].value, param_value_p, strlen(param_value_p));

      /*list_parameters[i].name = param_name_p;
      list_parameters[i].value = param_value_p;*/

      PRINTF("APPCFG: File '%s', value=%s\n", param_name_p, param_value_p);
      //PRINTF("APPCFG: list_parameters '%s', value=%s\n", list_parameters[i].name, list_parameters[i].value);

      i++;
    }
  }
  cfs_closedir(&dir);
  return NULL;
}
#endif


#if MAYBE_SOMEDAY
char*
app_config_get_parameter(const char* file, const char* param)
{
  static int fd = 0;
  int len;
  char buf[MAX_BLOCKSIZE];
  int offset = 0;
  char* pos;
  volatile char *param_p = NULL;
  char* param_value_p = NULL;
  static char param_value[MAX_PARAM_VALUE_LEN];
  size_t param_value_len;

  fd = cfs_open(file, CFS_READ);

  while(1){
    cfs_seek(fd, offset, CFS_SEEK_SET);
    len = cfs_read(fd, buf, MAX_BLOCKSIZE);

    if(len <= 0) {
      cfs_close(fd);
      break;
    } else {
      param_p = strstr(buf, param);
      if(param_p){

        // the parameter is found but its value may be cut in the middle,
        // re-read the entire line from the parameter
        cfs_seek(fd, (int)(param_p-buf)+offset, CFS_SEEK_SET);
        len = cfs_read(fd, buf, MAX_BLOCKSIZE);

        param_value_p = strchr(buf, '=')+1;
        param_value_len = (size_t)(strchr(param_value_p, '\n')-param_value_p);
        strncpy(param_value, param_value_p, param_value_len);
        param_value[param_value_len] = '\0';

        cfs_close(fd);
        break;
      }
      pos = strchr(buf, '\n');
      offset += (int)(pos-buf)+1;
    }
  }

  return param_value;

}

int32_t
app_config_edit_parameter(const char* file, const char* param, const char* value)
{
  static int fd = 0;
  int len;
  char buf[MAX_BLOCKSIZE];
  int offset = 0;
  char* pos;
  volatile char *param_p = NULL;

  fd = cfs_open(file, CFS_READ|CFS_WRITE);

  while(1){
    cfs_seek(fd, offset, CFS_SEEK_SET);
    len = cfs_read(fd, buf, MAX_BLOCKSIZE);

    if(len <= 0) {
      cfs_close(fd);
      break;
    } else {
      param_p = strstr(buf, param);
      if(param_p){

        // position the cursor at the begining of the parameter value
        cfs_seek(fd, (int)(param_p-buf)+offset+strlen(param)+1, CFS_SEEK_SET);
        cfs_write(fd, value, strlen(value));
        cfs_write(fd, "\n", 1);

        cfs_close(fd);
        break;
      }
      pos = strchr(buf, '\n');
      offset += (int)(pos-buf)+1;
    }
  }

  return 1;

}
#endif /* MAYBE_SOMEDAY */
