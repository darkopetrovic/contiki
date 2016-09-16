PROJECT = smart-sensor

WITH_SHELL=1
WITH_CUSTOMRDC=0
WITH_APPCONFIG=1
WITH_COAPSERVER=1
WITH_POWERTRACK=1

PROJECTDIR=..
CONTIKI = ../../..

ifeq ($(WITH_COAPSERVER),1)
  PROJECTDIRS += resources
  PROJECT_SOURCEFILES += res-sensors.c res-temp.c res-hum.c res-motion.c
  PROJECT_SOURCEFILES += res-power.c res-battery.c res-solar.c
  ifeq ($(WITH_APPCONFIG),1)
    PROJECT_SOURCEFILES += res-config.c
  endif
  ifeq ($(WITH_POWERTRACK),1)
    PROJECT_SOURCEFILES += res-energest.c
  endif
endif

include $(PROJECTDIR)/Makefile

