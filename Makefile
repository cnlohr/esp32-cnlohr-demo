PROJECT_NAME := esp32-cnlohr-demo

include $(IDF_PATH)/make/project.mk

CPPFLAGS += -DXT_INTEXC_HOOKS -DXCHAL_HAVE_NMI 

program.lst : build/$(PROJECT_NAME).elf
	xtensa-esp32-elf-objdump -S build/$(PROJECT_NAME).elf  > program.lst

