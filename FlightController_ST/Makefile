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
ASFLAGS  = $(CPU) $(OPT)

LDSCRIPT = STM32F103CBT6_FLASH.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(CPU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS  := $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
OBJECTS  += $(addprefix $(BUILD_DIR)/,$(notdir $(AS_SOURCES:.s=.o)))

vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
vpath %.s $(sort $(dir $(AS_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	$(CP) -O binary $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
