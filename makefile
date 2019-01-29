PROJNAME = robomecanum
TARGET   = STM32F407VG

SHELL    = /bin/bash

SRCDIR   = ./src ./stm32_drv/drv/src ./mculib/src
BUILDDIR = ./build
INCDIR   = ./inc
INCDIR  += ./stm32_drv/cmsis ./stm32_drv/hal ./stm32_drv/drv/inc
INCDIR  += ./mculib/inc

CC       = arm-none-eabi-gcc
OBJCOPY  = arm-none-eabi-objcopy
OBJSIZE  = arm-none-eabi-size

CFLAGS   = -std=c99 -Wall -Og -ggdb -flto
CFLAGS  += -mcpu=$(call find_core,$(TARGET))
CFLAGS  += -mthumb
CFLAGS  += -mfloat-abi=soft
CFLAGS  += $(addprefix -I,$(INCDIR))
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += $(call find_target,$(TARGET))

LDSCRIPT = mylinker.ld
LDFLAGS  = -T$(LDSCRIPT) -nostartfiles -ggdb
LDFLAGS += -Wl,--gc-sections

DEPFLAGS = -MT $@ -MMD -MP -MF $(BUILDDIR)/$*.Td

SRC_C    = $(wildcard $(addsuffix /*.c, $(SRCDIR)))
SRC_S    = $(wildcard $(addsuffix /*.S, $(SRCDIR)))

OBJ      = $(addprefix $(BUILDDIR)/, $(notdir $(SRC_S:.S=.o)))
OBJ     += $(addprefix $(BUILDDIR)/, $(notdir $(SRC_C:.c=.o)))

ELF      = $(addsuffix .elf, $(BUILDDIR)/$(PROJNAME))
BIN      = $(addsuffix .bin, $(BUILDDIR)/$(PROJNAME))

POSTCC   = @mv -f $(BUILDDIR)/$*.Td $(BUILDDIR)/$*.d && touch $@

$(shell mkdir -p $(BUILDDIR))

vpath %.c $(SRCDIR)
vpath %.S $(SRCDIR)

all: $(BIN)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	$(OBJSIZE) $<

$(ELF): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

$(BUILDDIR)/%.o: %.S
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/%.o : %.c
$(BUILDDIR)/%.o : %.c $(BUILDDIR)/%.d
	$(CC) $(DEPFLAGS) $(CFLAGS) -c $< -o $@
	$(POSTCC)

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)

.PHONY: dump
dump:
	arm-none-eabi-objdump -d $(ELF) | less

.PHONY: ddump
ddump:
	arm-none-eabi-objdump -D $(ELF) | less

.PHONY: flash
flash: $(BIN)
	st-flash write $(BIN) 0x08000000

.PHONY: print
print:
	@echo $(SRC_C)
	@echo $(SRC_S)

define find_target
$(shell 
mcu_families=(
    "STM32F030x6"
    "STM32F030x8"
    "STM32F031x6"
    "STM32F038xx"
    "STM32F042x6"
    "STM32F048xx"
    "STM32F051x8"
    "STM32F058xx"
    "STM32F070x6"
    "STM32F070xB"
    "STM32F071xB"
    "STM32F072xB"
    "STM32F078xx"
    "STM32F091xC"
    "STM32F098xx"
    "STM32F030xC"
    "STM32F407xx"
    );
for mcu in "$${mcu_families[@]}"; do
    this_regex="^$$mcu.*$$";
    this_regex=$${this_regex//x/[A-Z]};
    symbol=-D$$mcu;
    if [[ "$(1)" =~ $${this_regex} ]]; then
        echo $${symbol};
        echo -DSTM32_SERIES_$${mcu:5:2};
        exit;
    fi 
done;
echo NO_MCU_FOUND;
)
endef

define find_core
$(shell
    target=$(1);
    series=$${target:5:2};
    if [[ "$$series" == "F0" ]]; then
        echo cortex-m0;
    elif [[ "$$series" == "F4" ]]; then
        echo cortex-m4;
    else
        echo NO_CORE_FOUND;
    fi
)
endef

$(BUILDDIR)/%.d: ;
.PRECIOUS: $(BUILDDIR)/%.d

include $(wildcard $(patsubst %,$(BUILDDIR)/*.d,$(basename $(SRC_C))))
