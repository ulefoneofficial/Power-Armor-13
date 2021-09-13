#!/bin/sh

adb wait-for-device
adb root
adb wait-for-device
adb remount
adb wait-for-device

adb push aw881xx_pid_xx_spk_reg.bin vendor/firmware
adb push aw881xx_pid_xx_voice_reg.bin vendor/firmware
adb push aw881xx_pid_xx_fm_reg.bin vendor/firmware
adb push aw881xx_pid_xx_rcv_reg.bin vendor/firmware

adb push aw881xx_pid_01_spk_fw.bin vendor/firmware
adb push aw881xx_pid_01_spk_cfg.bin vendor/firmware
adb push aw881xx_pid_01_voice_fw.bin vendor/firmware
adb push aw881xx_pid_01_voice_cfg.bin vendor/firmware
adb push aw881xx_pid_01_fm_fw.bin vendor/firmware
adb push aw881xx_pid_01_fm_cfg.bin vendor/firmware
adb push aw881xx_pid_01_rcv_fw.bin vendor/firmware
adb push aw881xx_pid_01_rcv_cfg.bin vendor/firmware

adb push aw881xx_pid_03_spk_fw.bin vendor/firmware
adb push aw881xx_pid_03_spk_cfg.bin vendor/firmware
adb push aw881xx_pid_03_voice_fw.bin vendor/firmware
adb push aw881xx_pid_03_voice_cfg.bin vendor/firmware
adb push aw881xx_pid_03_fm_fw.bin vendor/firmware
adb push aw881xx_pid_03_fm_cfg.bin vendor/firmware
adb push aw881xx_pid_03_rcv_fw.bin vendor/firmware
adb push aw881xx_pid_03_rcv_cfg.bin vendor/firmware


