import BigCreteSpi as theCrete
<<<<<<< HEAD
import 4671RegDict as d
=======
>>>>>>> 4a12961b533b27fa99cc2f88e2566ce2da8b4f02
import time
import os

Lspi4671 = theCrete.BIG_CRETE_SPI(13, 12, 11, 16)
Lspi6100 = theCrete.BIG_CRETE_SPI(13, 12, 11, 7)
Rspi4671 = theCrete.BIG_CRETE_SPI(36, 38, 37, 40)
Rspi6100 = theCrete.BIG_CRETE_SPI(36, 38, 37, 35)

<<<<<<< HEAD
Lspi4671.writeByte(d.regs_4671["TMC4671_MOTOR_TYPE_N_POLE_PAIRS"],  [0x00,0x03,0x00,0x0F])
Lspi4671.writeByte(d.regs_4671["TMC4671_PWM_POLARITIES"],           [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(d.regs_4671["TMC4671_PWM_MAXCNT"],               [0x00,0x00,0x0F,0x9F])
Lspi4671.writeByte(d.regs_4671["TMC4671_PWM_BBM_H_BBM_L"],          [0x00,0x00,0x28,0x28])
Lspi4671.writeByte(d.regs_4671["TMC4671_PWM_SV_CHOP"],              [0x00,0x00,0x00,0x07])

#ADC configuration
Lspi4671.writeByte(d.regs_4671["TMC4671_ADC_I_SELECT"],             [0x18,0x00,0x01,0x00])
Lspi4671.writeByte(d.regs_4671["TMC4671_dsADC_MCFG_B_MCFG_A"],      [0x00,0x10,0x00,0x10])
Lspi4671.writeByte(d.regs_4671["TMC4671_dsADC_MCLK_A"],             [0x20,0x00,0x00,0x00])
Lspi4671.writeByte(d.regs_4671["TMC4671_dsADC_MCLK_B"],             [0x20,0x00,0x00,0x00])
Lspi4671.writeByte(d.regs_4671["TMC4671_dsADC_MDEC_B_MDEC_A"],      [0x01,0x4E,0x01,0x4E])
Lspi4671.writeByte(d.regs_4671["TMC4671_ADC_I0_SCALE_OFFSET"],      [0x01,0x00,0x81,0x3F])
Lspi4671.writeByte(d.regs_4671["TMC4671_ADC_I1_SCALE_OFFSET"],      [0x01,0x00,0x81,0xA8])

#ABN Encoder settings
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_MODE"],                 [0x00,0x00,0x10,0x00])
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_PPR"],                  [0x00,0x00,0x20,0x00])
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_COUNT"],                [0x00,0x00,0x18,0x94])
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])

# Limits
Lspi4671.writeByte(d.regs_4671["PID_TORQUE_FLUX_LIMITS"],           [0x00,0x00,0x03,0xE8])

# PI settings
Lspi4671.writeByte(d.regs_4671["PID_TORQUE_P_TORQUE_I"],            [0x01,0x00,0x01,0x00])
Lspi4671.writeByte(d.regs_4671["PID_FLUX_P_FLUX_I"],                [0x01,0x00,0x01,0x00])

# Encoder init
Lspi4671.writeByte(d.regs_4671["MODE_RAMP_MODE_MOTION"],            [0x00,0x00,0x00,0x08])
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_PHI_E_PHI_M_OFFSET"],   [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(d.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x01])
Lspi4671.writeByte(d.regs_4671["PHI_E_EXT"],                        [0x00,0x00,0x00,0x00])
Lspi4671.writeByte(d.regs_4671["UQ_UD_EXT"],                        [0x00,0x00,0x0F,0xA0])
time.sleep(1)
Lspi4671.writeByte(d.regs_4671["ABN_DECODER_COUNT"],                [0x00,0x00,0x00,0x00])

#Feedback selection
Lspi4671.writeByte(d.regs_4671["PHI_E_SELECTION"],                  [0x00,0x00,0x00,0x03])
Lspi4671.writeByte(d.regs_4671["VELOCITY_SELECTION"],               [0x00,0x00,0x00,0x09])

# Torque mode
Lspi4671.writeByte(d.regs_4671["MODE_RAMP_MODE_MOTION"],            [0x00,0x00,0x00,0x01])

# Rotate right
Lspi4671.writeByte(d.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0x03,0xE8,0x00,0x00])
time.sleep(1)

# Rotate left
Lspi4671.writeByte(d.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0xFC,0x18,0x00,0x00])
time.sleep(1)

#Stop
Lspi4671.writeByte(d.regs_4671["PID_TORQUE_FLUX_TARGET"],           [0x00,0x00,0x00,0x00])
=======

>>>>>>> 4a12961b533b27fa99cc2f88e2566ce2da8b4f02
