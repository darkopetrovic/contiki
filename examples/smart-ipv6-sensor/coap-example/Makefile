PROJECT = coap-example

WITH_SHELL=1
WITH_CUSTOMRDC=0
WITH_COAPSERVER=1

PROJECTDIR=..
CONTIKI = ../../..

ifeq ($(WITH_COAPSERVER),1)
  PROJECTDIRS += resources
  PROJECT_SOURCEFILES += res-temp.c
endif

include $(PROJECTDIR)/Makefile

