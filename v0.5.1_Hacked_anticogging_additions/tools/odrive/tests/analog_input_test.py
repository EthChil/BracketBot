
import test_runner

import time
import math
import os
import numpy as np

from odrive.enums import *
from test_runner import *


teensy_code_template = """
void setup() {
  analogWriteResolution(10);
  // base clock of the PWM timer is 150MHz (on Teensy 4.0)
  int freq = 150000000/1024; // ~146.5kHz PWM frequency
  analogWriteFrequency({analog_out}, freq);

  // for filtering, assuming we have a 150 Ohm resistor, we need a capacitor of
  // 1/(150000000/1024)*2*pi/150 = 2.85954744646751e-07 F, that's ~0.33uF

  //pinMode({lpf_enable}, OUTPUT);
}

int i = 0;
void loop() {
  i++;
  i = i & 0x3ff;
  if (digitalRead({analog_reset}))
    i = 0;
  analogWrite({analog_out}, i);
  delay(1);
}
"""


class TestAnalogInput():
    """
    Verifies the Analog input.

    The Teensy generates a PWM signal with a duty cycle that follows a sawtooth signal
    with a period of 1 second. The signal should be connected to the ODrive's
    analog input through a low-pass-filter.

                    ___                   ___
    Teensy PWM ----|___|-------o---------|___|----- ODrive Analog Input
                  150 Ohm      |        150 Ohm
                              === 
                               | 330nF
                               |
                              GND

    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            for odrive_gpio_num, odrive_gpio in [(2, odrive.gpio3), (3, odrive.gpio4)]:
                analog_out_options = []
                lpf_gpio = [gpio for lpf in testrig.get_connected_components(odrive_gpio, LowPassFilterComponent)
                                 for gpio in testrig.get_connected_components(lpf.en, LinuxGpioComponent)]
                for teensy_gpio in testrig.get_connected_components(odrive_gpio, TeensyGpio):
                    teensy = teensy_gpio.parent
                    analog_reset_options = []
                    for gpio in teensy.gpios:
                        for local_gpio in testrig.get_connected_components(gpio, LinuxGpioComponent):
                            analog_reset_options.append((gpio, local_gpio))
                    analog_out_options.append((teensy, teensy_gpio, analog_reset_options))
                yield (odrive, lpf_gpio, odrive_gpio_num, analog_out_options)


    def run_test(self, odrive: ODriveComponent, lpf_enable: LinuxGpioComponent, analog_in_num: int, teensy: TeensyComponent, teensy_analog_out: Component, teensy_analog_reset: Component, analog_reset_gpio: LinuxGpioComponent, logger: Logger):
        code = teensy_code_template.replace("{analog_out}", str(teensy_analog_out.num)).replace("{analog_reset}", str(teensy_analog_reset.num)) #.replace("lpf_enable", str(lpf_enable.num))
        teensy.compile_and_program(code)
        analog_reset_gpio.config(output=True)
        analog_reset_gpio.write(True)
        lpf_enable.config(output=True)
        lpf_enable.write(False)

        logger.debug("Set up analog input...")
        
        min_val = -20000
        max_val = 20000
        period = 1.025 # period in teensy code is 1s, but due to tiny overhead it's a bit longer

        analog_mapping = [
            None, #odrive.handle.config.gpio1_analog_mapping,
            None, #odrive.handle.config.gpio2_analog_mapping,
            odrive.handle.config.gpio3_analog_mapping,
            odrive.handle.config.gpio4_analog_mapping,
            None, #odrive.handle.config.gpio5_analog_mapping,
        ][analog_in_num]

        odrive.unuse_gpios()
        analog_mapping.endpoint = odrive.handle.axis0.controller._remote_attributes['input_pos']
        analog_mapping.min = min_val
        analog_mapping.max = max_val
        odrive.save_config_and_reboot()

        analog_reset_gpio.write(False)
        data = record_log(lambda: [odrive.handle.axis0.controller.input_pos], duration=5.0)

        # Expect mean error to be at most 2% (of the full scale).
        # Expect there to be less than 2% outliers, where an outlier is anything that is more than 5% (of full scale) away from the expected value.
        full_range = abs(max_val - min_val)
        slope, offset, fitted_curve = fit_sawtooth(data, min_val, max_val, sigma=30)
        test_assert_eq(slope, (max_val - min_val) / period, accuracy=0.005) 
        test_curve_fit(data, fitted_curve, max_mean_err = full_range * 0.02, inlier_range = full_range * 0.05, max_outliers = len(data[:,0]) * 0.02)



if __name__ == '__main__':
    test_runner.run(TestAnalogInput())
