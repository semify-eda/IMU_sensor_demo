"""
Demo script for the ASM330LHHXG1 IMU Sensor
"""
import numpy as np
import scipy.signal as signal
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from i2ct_debug import *
import time

import keyboard
import pyaudio
from multiprocessing import Process

from SmartWaveAPI import SmartWave
from imu_conf_lib import *
from sys import argv, exit
import csv
from timeit import default_timer

try:
    com = argv[1]
except:
    com = None

# If set to true, include the io-expander for visualization
IO_EXPANDER = False

def axl_conf(i2c, i2c_addr, odr: str = '12.5Hz', fs: str = '2g') -> None:
    """
    Method used to configure the accelerometer control register

    :param i2c: I2C object for SmartWave connection to the IMU sensor
    :param i2c_addr: Default I2C address of the ASM330LHHXG1 IMU Sensor
    :param odr: Accelerometer output data rate selection (in Hz)
    :param fs: Accelerometer full-scale selection.
    :return: None
    """
    axl_reg = ((axl_odr[odr] << 4) + (axl_fs[fs] << 2))
    i2c.writeRegister(i2c_addr, 0x10.to_bytes(1, 'big'), axl_reg.to_bytes(1, 'big'), timeout=None)
    ctrl1_xl = i2c.readRegister(i2c_addr, 0x10.to_bytes(1, 'big'), 1, timeout=None)
    print(f"Accelerometer control register value: {ctrl1_xl[0]:08b}")


def gyro_conf(i2c, i2c_addr, odr: str = '12.5Hz', fs_g: str = '250_dps',
              fs_125: str = 'fs_g', fs_4000: str = '4000_dps') -> None:
    """
    Method used to configure the gyroscope control register

    :param i2c: I2C object for SmartWave connection to the IMU sensor
    :param i2c_addr: Default I2C address of the ASM330LHHXG1 IMU Sensor
    :param odr: Gyroscope output data rate selection (in Hz)
    :param fs_g: Gyroscope chain full-scale selection
    :param fs_125: Selects gyroscope chain full-scale ±125 dps
    :param fs_4000: Selects gyroscope chain full-scale ±4000 dps
    :return: None
    """
    gyro_reg = ((gyro_odr[odr] << 4) + (gyro_fs_g[fs_g] << 2) +
                (gyro_fs_125[fs_125] << 1) + gyro_fs_4000[fs_4000])
    i2c.writeRegister(i2c_addr, 0x11.to_bytes(1, 'big'), gyro_reg.to_bytes(1, 'big'), timeout=None)
    ctrl2_g = i2c.readRegister(i2c_addr, 0x11.to_bytes(1, 'big'), 1, timeout=None)
    print(f"Gyroscope control register value: {ctrl2_g[0]:08b}")


def twos_comp(val, bits: int = 16):
    """
    Compute the two's complement of the given register value

    :param val: Input data
    :param bits: Bit length
    :return: Two's complement of the input data
    """
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val


def main():
    """
    The main function handles the connection to SmartWave and the ASM330LHHXG1 IMU Sensor. It configures the
    accelerometer and gyroscope, sets default plotting parameters, and reads the angular and linear rate registers
    for data visualization.
    :return: None
    """
    with SmartWave().connect(reset=False, port_name=com) as sw:
        with open("imu_data.csv", "w") as csvfile:

            csv_fields = ['x linear', 'y linear', 'z linear', 'x gyro', 'y gyro', 'z gyro']

            writer = csv.writer(csvfile)
            writer.writerow(csv_fields)
                
            sw.debugCallback = lambda x : print(x)
            i2c_imu_addr = 0x6a  # Default I2C address
            i2c_imu = sw.createI2CConfig(sda_pin_name="A2", scl_pin_name="A3", clock_speed=int(400e3))
            try:
                imu_id = i2c_imu.readRegister(i2c_imu_addr, 0x0f.to_bytes(1, 'big'), 1)
            except:
                try:
                    x = sw.readFPGARegister(0x880F0)
                    debug_reg_printout(x)
                except:
                    print("Unable to read contents of I2CT debug register...")
                SmartWave.disconnect(sw)
                exit("Unable to establish connection with IMU device on address 0x6a...")

            if IO_EXPANDER:
                i2c_io_exp_addr = 0x20
                i2c_io_exp = sw.createI2CConfig(sda_pin_name="A10", scl_pin_name="A9", clock_speed=int(400e3))
                i2c_io_exp.write(i2c_io_exp_addr, [0xff, 0xff])

            # Check if connection to the target device was successful
            if (imu_id[0] == 0xff) or (imu_id[0] == 0x00):
                exit("Couldn't reach device. Terminating code.")
            else:
                print(f"Connection was successful. Device ID: {imu_id[0]:#0x}")

            # Configure the ASM330LHHXG1 IMU
            axl_conf(i2c_imu, i2c_imu_addr)
            gyro_conf(i2c_imu, i2c_imu_addr)

            while(1):
                """
                This function is used to animate the plotting, which enables displaying real-time data.
                :param i: Placeholder
                :param ys: List that contains data for plotting the linear rate of change
                :return: Data for plotting
                """

                try:

                    pitch_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x22.to_bytes(1, 'big'), 1)
                    pitch_msb = i2c_imu.readRegister(i2c_imu_addr, 0x23.to_bytes(1, 'big'), 1)
                    # pitch_lsb = bytes([0]) 
                    # pitch_msb =  bytes([0]) 
                    pitch = (pitch_msb[0] << 8) + pitch_lsb[0]
                    tc_pitch = twos_comp(pitch)

                    roll_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x24.to_bytes(1, 'big'), 1)
                    roll_msb = i2c_imu.readRegister(i2c_imu_addr, 0x25.to_bytes(1, 'big'), 1)
                    # roll_lsb = bytes([0]) 
                    # roll_msb = bytes([0]) 
                    roll = (roll_msb[0] << 8) + roll_lsb[0]
                    tc_roll = twos_comp(roll)

                    yaw_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x26.to_bytes(1, 'big'), 1)
                    yaw_msb = i2c_imu.readRegister(i2c_imu_addr, 0x27.to_bytes(1, 'big'), 1)
                    # yaw_lsb = bytes([0]) 
                    # yaw_msb = bytes([0]) 
                    yaw = (yaw_msb[0] << 8) + yaw_lsb[0]
                    tc_yaw = twos_comp(yaw)

                    # Angular rate conversion - returns the rate of change
                    # print(f"Pitch: {tc_pitch:.3f}     Roll: {tc_roll:.3f}    Yaw: {tc_yaw:.3f}")
                    # print(f"Pitch: {pitch_res:.3f} degrees    Roll: {roll_res:.3f} degrees   Yaw: {yaw_res:.3f} degrees\n")

                    # Linear acceleration sensor
                    x_axis_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x28.to_bytes(1, 'big'), 1)
                    x_axis_msb = i2c_imu.readRegister(i2c_imu_addr, 0x29.to_bytes(1, 'big'), 1)
                    x_axis = (x_axis_msb[0] << 8) + x_axis_lsb[0]
                    tc_x_axis = twos_comp(x_axis, 16)

                    y_axis_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x2A.to_bytes(1, 'big'), 1)
                    y_axis_msb = i2c_imu.readRegister(i2c_imu_addr, 0x2B.to_bytes(1, 'big'), 1)
                    # y_axis_lsb = bytes([0]) 
                    # y_axis_msb = bytes([0]) 
                    y_axis = (y_axis_msb[0] << 8) + y_axis_lsb[0]
                    tc_y_axis = twos_comp(y_axis, 16)

                    z_axis_lsb = i2c_imu.readRegister(i2c_imu_addr, 0x2C.to_bytes(1, 'big'), 1)
                    z_axis_msb = i2c_imu.readRegister(i2c_imu_addr, 0x2D.to_bytes(1, 'big'), 1)
                    # z_axis_lsb = bytes([0])  
                    # z_axis_msb = bytes([0]) 
                    z_axis = (z_axis_msb[0] << 8) + z_axis_lsb[0]
                    tc_z_axis = twos_comp(z_axis, 16)

                    # Linear acceleration conversion
                    # convert from g to m/s^2 1 g-unit = 9.80665
                    x_res = tc_x_axis * axl_sens['2g'] * 9.80665
                    y_res = tc_y_axis * axl_sens['2g'] * 9.80665
                    z_res = tc_z_axis * axl_sens['2g'] * 9.80665
                    # print(f"X_a: {x_res:.3f} m/s^2    Y_a: {y_res:.3f} m/s^2   Z_a: {z_res:.3f} m/s^2\n")
                    # print(f"x axis: {hex(x_axis)}    y axis: {hex(y_axis)}    z axis: {hex(z_axis)}")

                    sample = ['0x%04x' % x_axis, '0x%04x' % y_axis, '0x%04x' % z_axis, '0x%04x' % pitch, '0x%04x' % roll, '0x%04x' % yaw]
                    
                    print(f"pitch: {'0x%04x' % pitch}    roll: {'0x%04x' % roll}    yaw: {'0x%04x' % yaw}")

                    if keyboard.is_pressed('w'):
                        try:
                            writer.writerow(sample)
                        except:
                            pass
                    if keyboard.is_pressed('q'):
                        try:
                            csvfile.close()
                        except:
                            pass

                    if IO_EXPANDER:
                        io_led_toggle(i2c_io_exp, i2c_io_exp_addr, y_res, x_res)

                except Exception as e:
                    print("Connection to IMU unit has been interrupted. Terminating Program.")
                    print(e)
                    print('\n\n\n')
                    try:
                        x = sw.readFPGARegister(0x880F0)
                        debug_reg_printout(x)

                    except:
                        print("Unable to read contents of I2CT debug register...")
                    SmartWave.disconnect(sw)
                    exit()

if __name__ == "__main__":
    process = Process(target=main)
    process.start()
    while process.is_alive():
        if keyboard.is_pressed('q'):
            print("Terminating code.")
            time.sleep(1.0)
            process.terminate()
            try:
                time.sleep(0.5)
                SmartWave.disconnect()
            except:
                pass
            break
