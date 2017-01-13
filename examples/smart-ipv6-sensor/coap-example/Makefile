PROJECT = coap-example

WITH_SHELL=0
WITH_CUSTOMRDC=1
WITH_APPCONFIG=0
WITH_COAPSERVER=1
WITH_POWERTRACK=0
WITH_SMARTLED=0

#TARGET_LIBFILES = -lc

#LDFLAGS += -Wl,-T msp430.x

PROJECTDIR=..
CONTIKI = ../../..

CONTIKI_TARGET_MAIN = contiki-z1-main-sleepy.c

ifeq ($(WITH_COAPSERVER),1)
  PROJECTDIRS += resources
  PROJECT_SOURCEFILES += res-sensors.c res-temp.c res-hum.c 
  ifeq ($(WITH_APPCONFIG),1)
    PROJECT_SOURCEFILES += res-config.c
  endif
  ifeq ($(WITH_POWERTRACK),1)
    PROJECT_SOURCEFILES += res-energest.c
  endif
endif


include $(PROJECTDIR)/Makefile

