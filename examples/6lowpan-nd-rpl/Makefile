CONTIKI_PROJECT = 6lowpan-nd-example
all: $(CONTIKI_PROJECT)

WITH_SHELL=1
WITH_UDPCLIENT=1
WITH_CUSTOMRDC=1

TARGET_NAME_HOST = 6lowpan-nd-host
TARGET_NAME_ROUTER = 6lowpan-nd-router

DEFINES=PROJECT_CONF_H=\"project-conf.h\"

# If the node is a HOST
ifeq ($(findstring $(TARGET_NAME_HOST),$(MAKECMDGOALS)),$(TARGET_NAME_HOST))
  DEFINES+=UIP_CONF_ROUTER=0

  # Test routing by sending packet with an udp-client.
  ifeq ($(WITH_UDPCLIENT),1)
    CFLAGS += -DUDPCLIENT=1
    PROJECTDIRS += ./apps/udp-client 
    PROJECT_SOURCEFILES += udp-client.c
  else
    CFLAGS += -DUDPCLIENT=0
  endif

  ifneq ($(WITH_UDP_CLIENT_AUTOSTART),0)
    CFLAGS += -DUDP_CLIENT_AUTOSTART=1
  endif
# If the node is a ROUTER
else
  DEFINES+=UIP_CONF_ROUTER=1
endif

ifeq ($(WITH_SHELL),1)
	APPS += serial-shell
	CFLAGS += -DSHELL=1
endif

# The router ignore this parameter within the code. 
ifeq ($(WITH_CUSTOMRDC),1)
	CFLAGS += -DCRDC_CONF_ENABLE_CUSTOM_RDC=1
else
	CFLAGS += -DCRDC_CONF_ENABLE_CUSTOM_RDC=0
endif

APPS += smart
APPS += smart-blink

SMALL=1

CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CONTIKI_WITH_RPL = 1
CONTIKI_WITH_6LOWPAN_ND = 1

include $(CONTIKI)/Makefile.include

$(TARGET_NAME_HOST).$(TARGET): $(CONTIKI_PROJECT) $(TARGET_NAME_HOST)-rename
$(TARGET_NAME_ROUTER).$(TARGET): $(CONTIKI_PROJECT) $(TARGET_NAME_ROUTER)-rename

$(TARGET_NAME_HOST)-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) $(TARGET_NAME_HOST).$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)

$(TARGET_NAME_ROUTER)-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) $(TARGET_NAME_ROUTER).$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)