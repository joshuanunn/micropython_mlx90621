# MicroPython class for the I2C Melexis MLX90621 16x4 thermopile array
# Tested on the MicroPython Pyboard v1.1
# 
# Written in 2019 by Joshua Nunn
# 
# Based upon the C implementation 'mlxd' by Chuck Werbick 2015
# https://github.com/alphacharlie/mlxd
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

import time

# MLX90621 default I2C addresses
MLX90621_I2C_EEPR = 0x50
MLX90621_I2C_READ = 0x60

class MLX90621:
    
    def __init__(self, refresh_rate=8, i2c=None):
        """
        Create instance of mlx90621 sensor class
        Requires refresh_rate argument in Hz (default is 8 Hz) and i2c object
        """
        self.i2c = i2c
        self.rate = refresh_rate
        # Create bytearrays for storing sensor related values
        self.EEPROM = bytearray(256)
        self.config = bytearray(2)
        self.PTAT_BYTES = bytearray(2)
        self.VCP_BYTES = bytearray(2)
        self.ir_pixels = bytearray(128)
        # Create two temperature arrays, arranged as 16 x 4 arrays to match sensor array
        # Array layout/referencing as defined in Pixel Position diagram in mlx90621 datasheet
        self.temperatures_int = [[0] * 16 for i in range(4)]
        self.temperatures_flt = [[0.0] * 16 for i in range(4)]
    
    def _uint16_t(self, number):
        imin = 0
        imax = 65535
        if number < imin:
            number = int(number + (1 + imax - imin))
        elif number > imax:
            number = int(number - (1 + imax - imin))
        return int(number)
    
    def _int16_t(self, number):
        imin = -32768
        imax = 32767
        if number < imin:
            number = int(number + (1 + imax - imin))
        elif number > imax:
            number = int(number - (1 + imax - imin))
        return int(number)
    
    def _uint8_t(self, number):
        imin = 0
        imax = 255
        if number < imin:
            number = int(number + (1 + imax - imin))
        elif number > imax:
            number = int(number - (1 + imax - imin))
        return int(number)
    
    def _int8_t(self, number):
        imin = -128
        imax = 127
        if number < imin:
            number = int(number + (1 + imax - imin))
        elif number > imax:
            number = int(number - (1 + imax - imin))
        return int(number)
    
    def _twos_16(self, high_byte, low_byte):
        return self._int16_t(self._uint16_t(256 * high_byte + low_byte))
    
    def _mlx90621_read_eeprom(self):
        """
        Read entire mlx90621 EEPROM
        """
        self.i2c.readfrom_mem_into(MLX90621_I2C_EEPR, 0x00, self.EEPROM)
    
    def _mlx90621_write_trim(self, trim_val):
        """
        Write the oscillator trimming value
        """
        trim = bytes([0x00, trim_val]) # MSB = 0x00, LSB = trim_val
        trim_check_lsb = self._uint8_t(trim[1] - 0xAA)
        trim_check_msb = self._uint8_t(trim[0] - 0xAA)
        write_trim = bytes([0x04, trim_check_lsb, trim[1], trim_check_msb, trim[0]])
        self.i2c.writeto(MLX90621_I2C_READ, write_trim)
    
    def _mlx90621_write_config(self, lsb, msb):
        """
        Write configuration
        """
        lsb_check = self._uint8_t(lsb - 0x55)
        msb_check = self._uint8_t(msb - 0x55)
        write_config = bytes([0x03, lsb_check, lsb, msb_check, msb])
        self.i2c.writeto(MLX90621_I2C_READ, write_config)
    
    def _mlx90621_read_config(self):
        """
        Read configuration
        """
        read_config = bytes([0x02, 0x92, 0x00, 0x01])
        self.i2c.writeto(MLX90621_I2C_READ, read_config, False)
        self.i2c.readfrom_into(MLX90621_I2C_READ, self.config)
    
    def _mlx90621_set_refresh_hz(self, hz):
        """
        Set IR refresh rate in mlx90621
        """
        if hz ==  512:
            rate_bits = 0b0000
        elif hz ==  256:
            rate_bits = 0b0110
        elif hz ==  128:
            rate_bits = 0b0111
        elif hz ==  64:
            rate_bits = 0b1000
        elif hz ==  32:
            rate_bits = 0b1001
        elif hz ==  16:
            rate_bits = 0b1010
        elif hz ==  8:
            rate_bits = 0b1011
        elif hz ==  4:
            rate_bits = 0b1100
        elif hz ==  2:
            rate_bits = 0b1101
        elif hz ==  1:
            rate_bits = 0b1110
        elif hz ==  0:
            rate_bits = 0b1111 # 0.5 Hz
        else:
            rate_bits = 0b1011 # Default to 8 Hz if incorrect value 
        self._mlx90621_read_config()
        config_msb = 0b01000110
        config_lsb = (0b0011 << 4 | rate_bits)
        self._mlx90621_write_config(config_lsb, config_msb)
    
    def _mlx90621_ptat(self):
        """
        Return PTAT (Proportional To Absolute Temperature)
        """
        read_ptat = bytes([0x02, 0x40, 0x00, 0x01])
        self.i2c.writeto(MLX90621_I2C_READ, read_ptat, False)
        self.i2c.readfrom_into(MLX90621_I2C_READ, self.PTAT_BYTES)
        return (self.PTAT_BYTES[1] << 8) | self.PTAT_BYTES[0]
    
    def _mlx90621_ta(self, resolution):
        """
        calculation of absolute chip temperature
        """
        ptat = self._mlx90621_ptat()
        resolution_comp = 2.0 ** (3 - resolution)
        k_t1_scale = self._int16_t((self.EEPROM[0xD2] & 0xF0) >> 4)
        k_t2_scale = self._int16_t((self.EEPROM[0xD2] & 0x0F) + 10)
        v_th = self._twos_16(self.EEPROM[0xDB], self.EEPROM[0xDA]) / resolution_comp
        k_t1 = self._twos_16(self.EEPROM[0xDD], self.EEPROM[0XDC]) / ((2 ** k_t1_scale) * resolution_comp)
        k_t2 = self._twos_16(self.EEPROM[0xDF], self.EEPROM[0xDE]) / ((2 ** k_t2_scale) * resolution_comp)
        return ((-k_t1 + (k_t1 * k_t1 - (4 * k_t2 * (v_th - ptat))) ** 0.5) / (2 * k_t2)) + 25.0
    
    def _mlx90621_cp(self):
        """
        Compensation pixel read
        """
        compensation_pixel_read = bytes([0x02, 0x41, 0x00, 0x01]) # Command: 0x02, Start address: 0x41, Address step: 0x00, number of reads: 0x01
        self.i2c.writeto(MLX90621_I2C_READ, compensation_pixel_read, False)
        self.i2c.readfrom_into(MLX90621_I2C_READ, self.VCP_BYTES)
        return ((self.VCP_BYTES[1] << 8) | self.VCP_BYTES[0])
    
    def _mlx90621_por(self):
        """
        Return POR/Brown-out flag
        """
        self._mlx90621_read_config()
        return ((self.config[1] & 0x04) == 0x04)
    
    def _mlx90621_ir_read(self):
        """
        IR data read
        """
        ir_whole_frame_read = bytes([0x02, 0x00, 0x01, 0x40]) # Command: 0x02, Start address: 0x00, Address step: 0x01, number of reads: 0x40
        self.i2c.writeto(MLX90621_I2C_READ, ir_whole_frame_read, False)
        self.i2c.readfrom_into(MLX90621_I2C_READ, self.ir_pixels)
    
    def mlx90621_init(self):
        """
        Initialise mlx90621
        """
        # Start initialisation routine - loops if calibration fails
        self.ta = -9999
        while (self.ta > 60.0 or self.ta < -40.0):
            time.sleep_us(50000) # Small delay to let things settle
            self._mlx90621_read_eeprom()
            self._mlx90621_write_trim(self.EEPROM[0xF7])
            self._mlx90621_write_config(self.EEPROM[0xF5], self.EEPROM[0xF6])    
            self._mlx90621_set_refresh_hz(self.rate)
            self._mlx90621_read_config()
            time.sleep_us(10000) # Small delay to let things settle
            resolution = ((self.config[1] << 8 | self.config[0]) & 0x30) >> 4
            self.ta = self._mlx90621_ta(resolution)
        # Derive To calculation parameters following successful initialisation
        self.resolution_comp = (2.0 ** (3 - resolution))
        self.emissivity = ((self.EEPROM[0xE5] << 8) | self.EEPROM[0xE4]) / 32768.0
        self.a_common = self._twos_16(self.EEPROM[0xD1], self.EEPROM[0xD0])
        self.a_i_scale = self._int16_t((self.EEPROM[0xD9] & 0xF0) >> 4)
        self.b_i_scale = self._int16_t(self.EEPROM[0xD9] & 0x0F)
        self.alpha_cp = self._uint16_t(((self.EEPROM[0xD7] << 8) | self.EEPROM[0xD6]) / ((2.0 ** self.EEPROM[0xE2]) * self.resolution_comp))
        self.a_cp = self._twos_16(self.EEPROM[0xD4], self.EEPROM[0xD3]) / self.resolution_comp
        self.b_cp = self._int8_t(self.EEPROM[0xD5]) / ((2.0 ** self.b_i_scale) * self.resolution_comp)
        self.tgc = self._int8_t(self.EEPROM[0xD8]) / 32.0
    
    def mlx90621_read_ir(self):
        """
        IR temperature calculation routine
        Call function after calling mlx90621_init()
        Populates self.temperatures_int array with IR temperatures in 10 x degC as integers (10ths of a degree)
        Populates self.temperatures_flt array with IR temperatures in degC as floats
        """
        # Check mlx90621 is ready and reinitialise if not
        while not self._mlx90621_por():
            time.sleep(1) # Delay to let things settle
            self.mlx90621_init()
        # Start IR read routine
        self._mlx90621_ir_read()
        vcp = self._int16_t(self._mlx90621_cp())
        for i in range(4):
            for j in range(16):
                ref_index = int((j * 4) + i) # Calculation reference index
                vir = self._int16_t(self.ir_pixels[ref_index * 2 + 1] << 8 | self.ir_pixels[ref_index * 2])
                ai = (self.a_common + self.EEPROM[ref_index] * (2.0 ** self.a_i_scale)) / self.resolution_comp
                bi = self._int8_t(self.EEPROM[0x40 + ref_index]) / ((2.0 ** self.b_i_scale) * self.resolution_comp)
                # Calculate To
                vcp_off_comp = vcp - (self.a_cp + (self.b_cp  * (self.ta - 25.0)))
                vir_off_comp = vir - (ai + (bi * (self.ta - 25.0)))
                vir_tgc_comp = vir_off_comp - self.tgc * vcp_off_comp
                vir_compensated = vir_tgc_comp / self.emissivity
                alpha_ij = ((self.EEPROM[0xE1] << 8) | self.EEPROM[0xE0]) / (2.0 ** self.EEPROM[0xE2])
                alpha_ij += (self.EEPROM[0x80 + ref_index] / (2.0 ** self.EEPROM[0xE3]))
                alpha_ij /= self.resolution_comp
                ksta = self._twos_16(self.EEPROM[0xE7], self.EEPROM[0xE6]) / (2.0 ** 20.0)
                alpha_comp = (1 + ksta * (self.ta - 25.0)) * (alpha_ij - self.tgc * self.alpha_cp)
                to = (((vir_compensated / alpha_comp) + ((self.ta + 273.15) ** 4)) ** 0.25) - 273.15
                # Write final temperatures to int and float output arrays
                self.temperatures_int[i][j] = int(to * 10.0) # Returned as integers (tenths of a degree)
                self.temperatures_flt[i][j] = to
