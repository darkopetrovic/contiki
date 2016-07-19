CONTIKI_PROJECT = 6lowpan-nd-example
DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CONTIKI_WITH_RPL = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NA=1
include $(CONTIKI)/Makefile.include
