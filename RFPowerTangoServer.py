import logging
import time

import numpy
import tango
from tango import DispLevel, AttrWriteType, DevState
from tango.server import attribute, command

from TangoServerPrototype import TangoServerPrototype
from TangoUtils import Configuration
from config_logger import config_logger
from log_exception import log_exception

t0 = time.time()


class RFPowerTangoServer(TangoServerPrototype):
    server_version = '0.0'
    server_name = 'Python RF Power Control Tango Server'
    device_list = []

    anode_power = attribute(label="anode_power", dtype=float,
                            display_level=DispLevel.OPERATOR,
                            access=AttrWriteType.READ,
                            unit="kW", format="%f",
                            doc="Tetrode anode power")

    power_limit = attribute(label="anode_power_limit", dtype=float,
                            display_level=DispLevel.OPERATOR,
                            access=AttrWriteType.READ_WRITE,
                            unit="kW", format="%f",
                            doc="Tetrode anode power limit")

    def init_device(self):
        self.power = 0.0
        self.rf_power = 0.0
        self.power_limit_value = 50.0
        self.device_name = ''
        self.timer = None
        self.adc = None
        self.dac = None
        super().init_device()
        self.power_limit_value = self.config.get('power_limit', 50.0)
        self.power_limit.set_write_value(self.power_limit_value)
        self.configure_tango_logging()
        RFPowerTangoServer.device_list.append(self)

    def set_config(self):
        super().set_config()
        try:
            self.device_name = self.get_name()
            self.set_state(DevState.INIT)
            self.set_status('Initialization')
            self.timer = tango.DeviceProxy(self.config.get('timer', 'binp/nbi/timing'))
            self.adc = tango.DeviceProxy(self.config.get('adc', 'binp/nbi/adc0'))
            self.dac = tango.DeviceProxy(self.config.get('dac', 'binp/nbi/dac0'))

            self.ia_scale = self.get_scale(self.adc, self.config.get('ia', 'chan1'))
            self.ea_scale = self.get_scale(self.adc, self.config.get('ea', 'chan2'))
            self.ua_scale = self.get_scale(self.adc, self.config.get('ua', 'chan3'))
            self.ic_scale = self.get_scale(self.adc, self.config.get('ic', 'chan4'))
            self.iscr_scale = self.get_scale(self.adc, self.config.get('iscr', 'chan5'))
            self.ug1_scale = self.get_scale(self.adc, self.config.get('ug1', 'chan6'))

            self.info('Initialized successfully')
            self.set_state(DevState.RUNNING)
            self.set_status('Initialized successfully')
        except Exception as ex:
            self.log_exception('Exception initializing')
            self.set_state(DevState.FAULT)
            self.set_status('Error initializing')
            return False
        return True

    def read_anode_power(self):
        return self.power

    def read_power_limit(self):
        return self.power_limit_value

    def write_power_limit(self, value):
        self.power_limit_value = value
        self.config['power_limit'] = value

    def get_scale(self, dp, name):
        config = dp.get_attribute_config_ex(name)[0]
        try:
            coeff = float(config.display_unit)
        except:
            coeff = 1.0
        return coeff

    @command(dtype_out=float)
    def calculate_anode_power(self):
        try:
            self.ia = self.adc.read_attribute(self.config.get('ia', 'chan1')).value * self.ia_scale
            self.ea = self.adc.read_attribute(self.config.get('ea', 'chan2')).value * self.ea_scale
            self.ua = self.adc.read_attribute(self.config.get('ua', 'chan3')).value * self.ua_scale
            self.ic = self.adc.read_attribute(self.config.get('ic', 'chan4')).value * self.ic_scale
            self.iscr = self.adc.read_attribute(self.config.get('iscr', 'chan5')).value * self.iscr_scale
            self.ug1 = self.adc.read_attribute(self.config.get('ug1', 'chan6')).value * self.ug1_scale
            try:
                t = numpy.arccos(-77.0/self.ug1)
                # a0 = (numpy.sin(t) - t * numpy.cos(t)) / (numpy.pi * (1 - numpy.cos(t)))
                a0 = (numpy.sin(t) - t * numpy.cos(t))
                # a1 = (t - numpy.sin(t) * numpy.cos(t)) / (numpy.pi * (1 - numpy.cos(t)))
                a1 = (t - numpy.sin(t) * numpy.cos(t))
                i1 = (self.ic - self.iscr) * a1 / a0
                prf = i1 * self.ua / 2.0
                self.rf_power = prf
                ptot = self.ea * self.ia
                pa = ptot - prf
                self.power = pa
                self.anode_power.set_quality(tango.AttrQuality.ATTR_VALID)
                return pa
            except:
                self.log_exception('Can not calculate power')
                self.power = -1.0
                self.rf_power = -1.0
                self.anode_power.set_quality(tango.AttrQuality.ATTR_INVALID)
                return -1.0
        except:
            self.anode_power.set_quality(tango.AttrQuality.ATTR_INVALID)
            self.log_exception('Error calculating power')
            return -1.0

    @command(dtype_in=str)
    def pulse_off(self, pwd):
        if pwd != 'topsecret':
            return
        n = 0
        for k in range(12):
            try:
                self.timer.write_attribute('channel_enable' + str(k), False)
            except:
                n +=1
            if n > 0:
                self.log_exception('Pulse off error')
            else:
                self.info('Pulse switched off')


def looping():
    global t0
    time.sleep(0.1)
    for dev in RFPowerTangoServer.device_list:
        time.sleep(0.001)
        try:
            p = dev.calculate_anode_power()
            if p > dev.power_limit_vaue:
                dev.error('Anode power limit exceeded')
                dev.pulse_off('topsecret')
        except:
            dev.log_exception('Error in loop')


if __name__ == "__main__":
    RFPowerTangoServer.run_server(event_loop=looping)
