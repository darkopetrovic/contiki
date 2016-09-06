all: $(PROJECT)

PROJECTDIR?=.

WITH_IPV6 ?= 1
WITH_RPL ?= 1
WITH_6LOWPAN_ND ?= 1

WITH_SMARTLED ?= 1
WITH_SHELL ?= 0
WITH_UDPCLIENT ?= 0
WITH_CUSTOMRDC ?= 0
WITH_COAPSERVER ?= 0

TARGET_NAME_HOST = $(PROJECT)-host
TARGET_NAME_ROUTER = $(PROJECT)-router

ifeq ($(findstring $(TARGET_NAME_HOST),$(MAKECMDGOALS)),$(TARGET_NAME_HOST))
	# If the node is a HOST
  DEFINES+=UIP_CONF_ROUTER=0
else
	# If the node is a ROUTER
  DEFINES+=UIP_CONF_ROUTER=1
  WITH_UDPCLIENT=0
endif

ifeq ($(WITH_SHELL),1)
	APPS += serial-shell
	CFLAGS += -DSHELL=1
endif

ifeq ($(WITH_CUSTOMRDC),1)
	CFLAGS += -DRDC_CONF_SLEEPING_HOST=1
	PROJECTDIRS += $(PROJECTDIR)/apps/custom-rdc 
  PROJECT_SOURCEFILES += custom-rdc.c
endif

# Test routing by sending packet with an udp-client.
ifeq ($(WITH_UDPCLIENT),1)
  CFLAGS += -DUDPCLIENT=1
  PROJECTDIRS += $(PROJECTDIR)/apps/udp-client 
  PROJECT_SOURCEFILES += udp-client.c
  ifneq ($(WITH_UDP_CLIENT_AUTOSTART),0)
    CFLAGS += -DUDP_CLIENT_AUTOSTART=1
  endif
else
  CFLAGS += -DUDPCLIENT=0
endif

ifeq ($(WITH_COAPSERVER),1)
	CFLAGS += -DREST=coap_rest_implementation
ifneq ($(TARGET),native)
	CFLAGS += -DUIP_CONF_TCP=0
endif
  APPS += er-coap
  APPS += rest-engine
endif

ifeq ($(WITH_SMARTLED),1)
  PROJECTDIRS += $(PROJECTDIR)/apps/smart-led 
  PROJECT_SOURCEFILES += smart-led.c
endif

SMALL=1

CONTIKI ?= ../..
CONTIKI_WITH_IPV6 = $(WITH_IPV6)
CONTIKI_WITH_RPL = $(WITH_RPL)
CONTIKI_WITH_6LOWPAN_ND = $(WITH_6LOWPAN_ND)

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

PROJECTDIRS += $(PROJECTDIR)

include $(CONTIKI)/Makefile.include

$(TARGET_NAME_HOST).$(TARGET): $(CONTIKI_PROJECT) $(TARGET_NAME_HOST)-rename
$(TARGET_NAME_ROUTER).$(TARGET): $(CONTIKI_PROJECT) $(TARGET_NAME_ROUTER)-rename

$(TARGET_NAME_HOST)-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) $(TARGET_NAME_HOST).$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)

$(TARGET_NAME_ROUTER)-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) $(TARGET_NAME_ROUTER).$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)