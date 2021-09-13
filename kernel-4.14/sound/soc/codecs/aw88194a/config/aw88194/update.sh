#!/bin/sh

adb wait-for-device
adb root
adb wait-for-device
adb remount
adb wait-for-device

cd mono
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
cd ..

cd left
adb push aw881xx_pid_xx_l_spk_reg.bin vendor/firmware
adb push aw881xx_pid_xx_l_voice_reg.bin vendor/firmware
adb push aw881xx_pid_xx_l_fm_reg.bin vendor/firmware
adb push aw881xx_pid_xx_l_rcv_reg.bin vendor/firmware

adb push aw881xx_pid_01_l_spk_fw.bin vendor/firmware
adb push aw881xx_pid_01_l_spk_cfg.bin vendor/firmware
adb push aw881xx_pid_01_l_voice_fw.bin vendor/firmware
adb push aw881xx_pid_01_l_voice_cfg.bin vendor/firmware
adb push aw881xx_pid_01_l_fm_fw.bin vendor/firmware
adb push aw881xx_pid_01_l_fm_cfg.bin vendor/firmware
adb push aw881xx_pid_01_l_rcv_fw.bin vendor/firmware
adb push aw881xx_pid_01_l_rcv_cfg.bin vendor/firmware
cd ..


cd right
adb push aw881xx_pid_xx_r_spk_reg.bin vendor/firmware
adb push aw881xx_pid_xx_r_voice_reg.bin vendor/firmware
adb push aw881xx_pid_xx_r_fm_reg.bin vendor/firmware
adb push aw881xx_pid_xx_r_rcv_reg.bin vendor/firmware

adb push aw881xx_pid_01_r_spk_fw.bin vendor/firmware
adb push aw881xx_pid_01_r_spk_cfg.bin vendor/firmware
adb push aw881xx_pid_01_r_voice_fw.bin vendor/firmware
adb push aw881xx_pid_01_r_voice_cfg.bin vendor/firmware
adb push aw881xx_pid_01_r_fm_fw.bin vendor/firmware
adb push aw881xx_pid_01_r_fm_cfg.bin vendor/firmware
adb push aw881xx_pid_01_r_rcv_fw.bin vendor/firmware
adb push aw881xx_pid_01_r_rcv_cfg.bin vendor/firmware
cd ..

