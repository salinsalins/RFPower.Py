import sys
import time

import numpy
import tango
from tango import DispLevel, AttrWriteType, DevState
from tango.server import attribute, command

sys.path.append('../TangoUtils')
from TangoServerPrototype import TangoServerPrototype

t0 = time.time()
OFF_PASSWORD = 'topsecret'


class RFPowerTangoServer(TangoServerPrototype):
    server_version_value = '1.0'
    server_name_value = 'Python RF Power Control Tango Server'
    device_list = []

    anode_power = attribute(label="anode_power", dtype=float,
                            display_level=DispLevel.OPERATOR,
                            access=AttrWriteType.READ,
                            unit="kW", format="%f",
                            doc="Tetrode anode power")

    anode_power_ok = attribute(label="anode_power_ok", dtype=bool,
                               display_level=DispLevel.OPERATOR,
                               access=AttrWriteType.READ,
                               unit="", format="%s",
                               doc="Is Tetrode anode power OK")

    output_power = attribute(label="output_power", dtype=float,
                             display_level=DispLevel.OPERATOR,
                             access=AttrWriteType.READ,
                             unit="kW", format="%f",
                             doc="Tetrode output power")

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
        # devices
        self.timer = None
        self.adc = None
        self.dac = None
        # attribute values
        self.ia = None
        self.ea = None
        self.ua = None
        self.ic = None
        self.iscr = None
        self.ug1 = None
        self.count = 0
        #
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

            self.ia_scale = self.get_scale(self.adc, self.config.get('ia', 'chan15'))
            self.ea_scale = self.get_scale(self.adc, self.config.get('ea', 'chan16'))
            self.ua_scale = self.get_scale(self.adc, self.config.get('ua', 'chan1'))
            self.ic_scale = self.get_scale(self.adc, self.config.get('ic', 'chan22'))
            self.iscr_scale = self.get_scale(self.adc, self.config.get('iscr', 'chan0'))
            self.ug1_scale = self.get_scale(self.adc, self.config.get('ug1', 'chan2'))

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

    def read_anode_power_ok(self):
        return self.power <= self.power_limit_value and self.get_state() == DevState.RUNNING

    def read_output_power(self):
        return self.rf_power

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
            if self.get_state() != DevState.RUNNING:
                self.warning('Server is not initialized')
                return
            self.ia = self.adc.read_attribute(self.config.get('ia', 'chan15')).value * self.ia_scale
            self.ea = self.adc.read_attribute(self.config.get('ea', 'chan16')).value * self.ea_scale
            self.ua = self.adc.read_attribute(self.config.get('ua', 'chan1')).value * self.ua_scale
            self.ic = self.adc.read_attribute(self.config.get('ic', 'chan22')).value * self.ic_scale
            self.iscr = self.adc.read_attribute(self.config.get('iscr', 'chan0')).value * self.iscr_scale
            self.ug1 = self.adc.read_attribute(self.config.get('ug1', 'chan2')).value * self.ug1_scale
            if numpy.abs(self.ug1) < 77.0:
                t = 1.0e-6
            else:
                t = numpy.arccos(-77.0 / self.ug1)
            # a0 = (numpy.sin(t) - t * numpy.cos(t)) / (numpy.pi * (1 - numpy.cos(t)))
            a0 = (numpy.sin(t) - t * numpy.cos(t))
            # a1 = (t - numpy.sin(t) * numpy.cos(t)) / (numpy.pi * (1 - numpy.cos(t)))
            a1 = (t - numpy.sin(t) * numpy.cos(t))
            i1 = (self.ic - self.iscr) * a1 / a0
            prf = i1 * self.ua / 2.0
            self.rf_power = prf
            self.output_power.set_value(self.rf_power)
            self.output_power.set_quality(tango.AttrQuality.ATTR_VALID)
            ptot = self.ea * self.ia
            pa = ptot - prf
            self.power = pa
            self.anode_power.set_value(self.power)
            self.anode_power.set_quality(tango.AttrQuality.ATTR_VALID)
            return pa
        except:
            self.log_exception('Can not calculate power')
            self.power = -1.0
            self.rf_power = -1.0
            self.output_power.set_value(self.rf_power)
            self.anode_power.set_value(self.power)
            self.anode_power.set_quality(tango.AttrQuality.ATTR_INVALID)
            self.output_power.set_quality(tango.AttrQuality.ATTR_INVALID)
            return -1.0

    @command(dtype_in=str)
    def pulse_off(self, pwd):
        if pwd != OFF_PASSWORD:
            self.debug('Incorrect password')
            return
        n = 0
        for k in range(12):
            try:
                self.timer.write_attribute('channel_enable' + str(k), False)
            except:
                n += 1
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
            if dev.get_state() != DevState.RUNNING:
                continue
            p = dev.calculate_anode_power()
            if p > dev.power_limit_value:
                dev.count += 1
                if dev.count >= 3:
                    dev.error('Anode power limit exceeded')
                    dev.pulse_off(OFF_PASSWORD)
            else:
                dev.count = 0
        except:
            dev.log_exception('Error in loop')


if __name__ == "__main__":
    RFPowerTangoServer.run_server(event_loop=looping)
    # RFPowerTangoServer.run_server()
