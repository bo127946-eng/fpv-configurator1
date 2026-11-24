TARGET = FlightController
BUILD_DIR = build

CXX = arm-none-eabi-g++
AS  = arm-none-eabi-as
CP  = arm-none-eabi-objcopy
SZ  = arm-none-eabi-size

CXX_SOURCES := $(wildcard *.cpp) $(wildcard */*.cpp)
AS_SOURCES  := $(wildcard *.s) $(wildcard */*.s)

CPU = -mcpu=cortex-m3 -mthumb
DEFS = -DSTM32F103xB -DUSE_FULL_LL_DRIVER
INCLUDES = -I.

OPT = -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti
CXXFLAGS = $(CPU) $(DEFS) $(INCLUDES) $(OPT) -std=c++17 -Wall -Wextra -Wno-unused-parameter -MMD -MP
ASFLAGS  = $(CPU)

LDSCRIPT = STM32F103CBT6_FLASH.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(CPU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

CXX_OBJS := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(notdir $(CXX_SOURCES))))
AS_OBJS  := $(addprefix $(BUILD_DIR)/,$(addsuffix .o,$(notdir $(AS_SOURCES))))
OBJECTS  := $(CXX_OBJS) $(AS_OBJS)

vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
vpath %.s $(sort $(dir $(AS_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.cpp.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.s.o: %.s Makefile | $(BUILD_DIR)
	$(AS) $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	$(CP) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
