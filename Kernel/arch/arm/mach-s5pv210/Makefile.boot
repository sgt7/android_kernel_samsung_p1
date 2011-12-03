zreladdr-y	:= 0x20008000
params_phys-y	:= 0x20000100

# override for C110
zreladdr-$(CONFIG_ARCH_S5PV210)	:= 0x30008000
params_phys-$(CONFIG_ARCH_S5PV210)	:= 0x30000100

