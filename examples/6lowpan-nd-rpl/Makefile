CONTIKI_PROJECT = 6lowpan-nd-example

TARGET_NAME_HOST = 6lowpan-nd-host
TARGET_NAME_ROUTER = 6lowpan-nd-router

DEFINES=PROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)

ifeq ($(findstring $(TARGET_NAME_HOST),$(MAKECMDGOALS)),$(TARGET_NAME_HOST))
  DEFINES+=UIP_CONF_ROUTER=0
else
  DEFINES+=UIP_CONF_ROUTER=1
endif

APPS = serial-shell
CFLAGS += -DSHELL=1

SMALL=1

CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CONTIKI_WITH_RPL = 1
include $(CONTIKI)/Makefile.include

6lowpan-nd-host.$(TARGET): $(CONTIKI_PROJECT) 6lowpan-nd-host-rename
6lowpan-nd-router.$(TARGET): $(CONTIKI_PROJECT) 6lowpan-nd-router-rename

6lowpan-nd-host-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) 6lowpan-nd-host.$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)

6lowpan-nd-router-rename:
	mv $(CONTIKI_PROJECT).$(TARGET) 6lowpan-nd-router.$(TARGET)
	rm -rf *.map contiki*.a obj_$(TARGET)