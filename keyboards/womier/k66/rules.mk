# project specific files
SRC = ../../../drivers/led/sn32/matrix_sn32f24xx.c
SRC += rgb_matrix.c #RGB Matrix driver
SRC += config_led.c #LED Matrix config
SRC += underglow.c  #to control underglow LEDs via I2C
SRC += rand.c		#replace rand() with something smaller

# MCU name
MCU = SN32F268F


# Build Options
#   comment out to disable the options.
#
LTO_ENABLE = yes			# reduces firmware size
MAGIC_ENABLE = yes			# enable magic, can be disabled to reduce size
SPACE_CADET_ENABLE = yes
GRAVE_ESC_ENABLE = yes
MOUSEKEY_ENABLE = no    	# Mouse keys
EXTRAKEY_ENABLE = yes   	# Audio control and System control
CONSOLE_ENABLE = no     	# Console for debug
COMMAND_ENABLE = no     	# Commands for debug and configuration
NKRO_ENABLE = yes        	# USB Nkey Rollover
KEYBOARD_SHARED_EP = no		# not needed, no = better BIOS support
AVR_USE_MINIMAL_PRINTF = yes
CUSTOM_MATRIX = yes			# custom key matrix driver, since COLs are shared with LED ON/OFF
RGB_MATRIX_ENABLE = yes		# use RGB Matrix
RGB_MATRIX_DRIVER = custom  # use LED custom driver, since COLs are shared

USE_EXCEPTIONS_STACKSIZE = 0xFF
USE_PROCESS_STACKSIZE = 0x250
# 0C0 lowest possible atm, 0xFF stable
# 0x170 lowest possible atm, 0x200 stable
