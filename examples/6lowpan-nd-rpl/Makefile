CONTIKI_PROJECT = 6lowpan-nd-example
DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)

APPS = serial-shell
CFLAGS += -DSHELL=1

SMALL=1

CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CONTIKI_WITH_RPL = 1
include $(CONTIKI)/Makefile.include
