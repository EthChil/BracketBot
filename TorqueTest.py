import sys
sys.path.append("home/bracketbot/Desktop/BracketBot/")
import BigCreteSpi as theCrete
import time
import os

import refridge as r

Lspi4671 = theCrete.BIG_CRETE_SPI(13, 12, 11, 16)
Lspi6100 = theCrete.BIG_CRETE_SPI(13, 12, 11, 7)
Rspi4671 = theCrete.BIG_CRETE_SPI(36, 38, 37, 40)
Rspi6100 = theCrete.BIG_CRETE_SPI(36, 38, 37, 35)

# LEFT SIDE
Lspi6100.writeByte(0x01, [255,255,255,255])
Lspi6100.writeByte(0x00, [00,00,00,00])
data = Lspi6100.readByte(0x0A)
data[0] = 00
data[1] = 00
Lspi6100.writeByte(0x0A, data)

Lspi4671.writeByte(r.regs_4671["MOTOR_TYPE_N_POLE_PAIRS"],  [0x00,0x03,0x00,0x0F])
Lspi4671.writeByte(r.regs_4671["PWM_POLARITIES"],           [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(r.regs_4671["PWM_MAXCNT"],               [0x00,0x00,0x0F,0x9F])
Lspi4671.writeByte(r.regs_4671["PWM_BBM_H_BBM_L"],          [0x00,0x00,0x28,0x28])
Lspi4671.writeByte(r.regs_4671["PWM_SV_CHOP"],              [0x00,0x00,0x00,0x07])

#ADC configuration
Lspi4671.writeByte(r.regs_4671["ADC_I_SELECT"],             [0x18,0x00,0x01,0x00])
Lspi4671.writeByte(r.regs_4671["dsADC_MCFG_B_MCFG_A"],      [0x00,0x10,0x00,0x10])
Lspi4671.writeByte(r.regs_4671["dsADC_MCLK_A"],             [0x20,0x00,0x00,0x00])
Lspi4671.writeByte(r.regs_4671["dsADC_MCLK_B"],             [0x20,0x00,0x00,0x00])
Lspi4671.writeByte(r.regs_4671["dsADC_MDEC_B_MDEC_A"],      [0x01,0x4E,0x01,0x4E])
Lspi4671.writeByte(r.regs_4671["ADC_I0_SCALE_OFFSET"],      [0x01,0x00,0x81,0x3F])
Lspi4671.writeByte(r.regs_4671["ADC_I1_SCALE_OFFSET"],      [0x01,0x00,0x81,0xA8])

#ABN Encoder settings
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_MODE"],                 [0x00,0x00,0x10,0x00])
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_PPR"],                  [0x00,0x00,0x20,0x00])
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_COUNT"],                [0x00,0x00,0x18,0x94])
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])

# Limits
Lspi4671.writeByte(r.regs_4671["PID_TORQUE_FLUX_LIMITS"],           [0x00,0x00,0x03,0xE8])

# PI settings
Lspi4671.writeByte(r.regs_4671["PID_TORQUE_P_TORQUE_I"],            [0x01,0x00,0x01,0x00])
Lspi4671.writeByte(r.regs_4671["PID_FLUX_P_FLUX_I"],                [0x01,0x00,0x01,0x00])

# Encoder init
print("Initializing Encoder...")
Lspi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"],            [0x00,0x00,0x00,0x08])
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(r.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x01])
Lspi4671.writeByte(r.regs_4671["PHI_E_EXT"],                        [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(r.regs_4671["UQ_UD_EXT"],                        [0x00,0x00,0x07,0xD0])
time.sleep(1)
Lspi4671.writeByte(r.regs_4671["ABN_DECODER_COUNT"],                [0x00,0x00,0x00,0x00])
print("Encoder Initialized")
#Feedback selection
Lspi4671.writeByte(r.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x03])
Lspi4671.writeByte(r.regs_4671["VELOCITY_SELECTION"],               [0x00,0x00,0x00,0x09])

# Torque mode
Lspi4671.writeByte(r.regs_4671["MODE_RAMP_MODE_MOTION"],            [0x00,0x00,0x00,0x01])
print("Rotate Right")
# Rotate right
Lspi4671.writeByte(r.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0x03,0xE8,0x00,0x00])
time.sleep(1)

# Rotate left
print("Rotate Left")
Lspi4671.writeByte(r.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0xFC,0x18,0x00,0x00])
time.sleep(1)

#Stop
Lspi4671.writeByte(r.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0x00,0x00,0x00,0x00])
time.sleep(1)

