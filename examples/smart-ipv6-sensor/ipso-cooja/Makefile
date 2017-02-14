PROJECT = ipso-cooja

WITH_SHELL=1
WITH_CUSTOMRDC=1
WITH_POWERTRACK=0
WITH_SMARTLED=0
WITH_OMA_LWM2M=1

PROJECTDIR=..
CONTIKI = ../../..

# Add the project specific resources
ifeq ($(WITH_OMA_LWM2M),1)
  PROJECTDIRS += ipso-objects
  PROJECT_SOURCEFILES += ipso-temperature.c ipso-humidity.c ipso-barometer.c ipso-illuminance.c
  PROJECT_SOURCEFILES += lwm2m-connectivity.c
  PROJECT_SOURCEFILES += ipso-objects.c
endif

include $(PROJECTDIR)/Makefile

