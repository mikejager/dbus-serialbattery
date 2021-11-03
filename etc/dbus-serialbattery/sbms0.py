# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function, unicode_literals
import sys

if sys.version_info.major == 2:
    import gobject
else:
    from gi.repository import GLib as gobject
import serial

from battery import Battery, Cell
import utils

BATTERY_CELL_BALANCE_ALARM = 100  # mV
BATTERY_CELL_BALANCE_WARN = 50    # mV

BATTERY_SOC_ALARM = 10            # %
BATTERY_SOC_WARN = 20             # %

BATTERY_TEMP_HIGH_ALARM = 40      # degrees C
BATTERY_TEMP_LOW_ALARM = 2        # degrees C

SBMS_SERIAL_INTERVAL = 30         # seconds

SBMS_STATUS_CODES = [
    'OV',    # over voltage
    'OVLK',  # error: cell voltage > over volt limit
    'UV',    # under voltage
    'UVLK',  # error: cell voltage < under volt limit
    'IOT',   # error: internal over-temperature
    'COC',   # error: charge over-current
    'DOC',   # error: discharge over-current
    'DSC',   # error: discharge short-circuit
    'CELF',  # error: cell failure (cell low-voltage)
    'OPEN',  # error: open cell wire (bad monitoring wire connection)
    'LVC',   # error: cell low-voltage cutoff
    'ECCF',  # error: EEPROM fail
    'CFET',  # charge FET enabled
    'EOC',   # end-of-charge
    'DFET',  # discharge FET enabled
]

SBMS_CELL_COUNT_MAX = 8
SBMS_CELL_COUNT_MAP = {
    3: (1, 2, 8),
    4: (1, 2, 7, 8),
    5: (1, 2, 3, 7, 8),
    6: (1, 2, 3, 6, 7, 8),
    7: (1, 2, 3, 4, 6, 7, 8),
    8: (1, 2, 3, 4, 5, 6, 7, 8)
}


def dcmp(comp, pos, size):
    # comp = compressed data
    # pos = location of desired data in the string
    # size = size of the desired data, e.g.
    #   - year is 1
    #   - cell voltage is 2
    #   - PV1 current is 3
    decomp = 0
    for i in range(size):
        decomp += (ord(comp[(pos + size - 1) - i]) - 35) * pow(91, i)
    return decomp


class Sbms0(Battery):

    def __init__(self, port, baud):
        super(Sbms0, self).__init__(port, baud)
        self.type = self.BATTERYTYPE

    BATTERYTYPE = "SBMS0"
    LENGTH_FIXED = 59

    def test_connection(self):
        # call a function that will connect to the battery, send a command and retrieve the result.
        # The result or call should be unique to this BMS. Battery name or version, etc.
        # Return True if success, False for failure
        with serial.Serial(self.port, self.baud_rate, timeout=SBMS_SERIAL_INTERVAL) as uart:
            return self.process_status_data(uart.readline())

    def get_settings(self):
        # After successful  connection get_settings will be call to set up the battery.
        # Set the current limits, populate cell count, etc
        # Return True if success, False for failure
        self.max_battery_current = utils.MAX_BATTERY_CURRENT
        self.max_battery_discharge_current = utils.MAX_BATTERY_DISCHARGE_CURRENT
        self.max_battery_voltage = utils.MAX_CELL_VOLTAGE * self.cell_count
        self.min_battery_voltage = utils.MIN_CELL_VOLTAGE * self.cell_count

        uart = serial.Serial(self.port, self.baud_rate)
        channel = gobject.IOChannel.unix_new(uart)
        gobject.io_add_watch(channel, gobject.PRIORITY_HIGH, gobject.IO_IN, self._uart_rx)
        return True

    def refresh_data(self):
        # call all functions that will refresh the battery data.
        # This will be called for every iteration (1 second)
        # Return True if success, False for failure

        # SBMS data is retrieved asynchronously, so always return success for
        # the polling function
        return True

    def _uart_rx(self, channel, condition):
        (status, buf, length, terminator_pos) = channel.read_line()
        line = buf.strip() if buf is not None else ''
        if not self.process_status_data(line):
            utils.logger.error("Didn't get the data expected")
            return False  # or maybe GLib.MainLoop.quit()?
        return True

    def process_status_data(self, line):
        if len(line) != self.LENGTH_FIXED:
            return False

        cell_voltages = [float(dcmp(line, 8 + cell * 2, 2) / 1000.0)
                         for cell in range(SBMS_CELL_COUNT_MAX)]
        if not self.cell_count:
            self.cells = [Cell(False) for c in range(SBMS_CELL_COUNT_MAX)]
            self.cell_count = len([v for v in cell_voltages if v > 0.0])
        for cell, voltage in enumerate(cell_voltages):
            if voltage > 0.0:
                self.cells[cell].voltage = voltage
        cell_voltage_delta = self.get_max_cell_voltage() - self.get_min_cell_voltage()

        flags = dcmp(line, 56, 3)
        status = {name: bool(flags & pow(2, i))
                  for i, name in enumerate(SBMS_STATUS_CODES)}

        self.soc = dcmp(line, 6, 2)
        self.voltage = round(sum(cell_voltages), 3)
        self.current = float(line[28] + str(dcmp(line, 29, 3))) / 1000.0

        self.temp1 = float(dcmp(line, 24, 2) - 450) / 10.0  # internal
        self.temp2 = float(dcmp(line, 26, 2) - 450) / 10.0  # external

        self.charge_fet = status['CFET']
        self.discharge_fet = status['DFET']

        self.protection.voltage_high = 2 if status['OVLK'] else 0
        self.protection.voltage_low = 2 if status['UVLK'] else 0
        self.protection.voltage_cell_low = 2 if status['LVC'] else 0
        self.protection.soc_low = 2 if self.soc < BATTERY_SOC_ALARM else 1 if self.soc < BATTERY_SOC_WARN else 0
        self.protection.current_over = 2 if status['COC'] else 0
        self.protection.current_under = 2 if status['DOC'] else 0
        self.protection.cell_imbalance = 2 if cell_voltage_delta > BATTERY_CELL_BALANCE_ALARM else 1 if cell_voltage_delta > BATTERY_CELL_BALANCE_WARN else 0
        self.protection.internal_failure = 2 if status['ECCF'] or status['OPEN'] else 0
        self.protection.temp_high_charge = 2 if status['IOT'] or (self.current > 0.0 and self.get_max_temp() >= BATTERY_TEMP_HIGH_ALARM) else 0
        self.protection.temp_low_charge = 2 if self.get_min_temp() <= BATTERY_TEMP_HIGH_ALARM else 0
        self.protection.temp_high_discharge = 2 if status['IOT'] or (self.current > 0.0 and self.get_max_temp() >= BATTERY_TEMP_HIGH_ALARM) else 0
        self.protection.temp_low_discharge = 2 if self.get_min_temp() <= BATTERY_TEMP_HIGH_ALARM else 0

        self.hardware_version = "SBMS0 " + str(self.cell_count) + " cells"
        utils.logger.info(self.hardware_version)
        return True
