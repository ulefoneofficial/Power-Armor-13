
# Fortsense 2-1: Please edit kernel-3.18/drivers/input/fingerprint/Kconfig to add FORTSENSE_FP option.
config FORTSENSE_FP
	tristate "Fortsense Fingerprint"
	default y
	help
	  If you say Y to this option, support will be included for 
	  the Fortsense's fingerprint sensor. This driver supports 
	  both REE and TEE. If in REE, CONFIG_SPI_SPIDEV must be set 
	  to use the standard 'spidev' driver.
	
	  This driver can also be built as a module. If so, the module
	  will be called 'fortsense_fp'.


# Fortsense 2-2: Please edit kernel-3.18/drivers/input/fingerprint/Makefile to add FORTSENSE_FP option.
obj-$(CONFIG_FORTSENSE_FP) += fortsense_fp/

