"""
Demo script for the ASM330LHHXG1 IMU Sensor
"""
import numpy as np
import scipy.signal as signal
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
from timeit import default_timer

import keyboard
import pyaudio
from multiprocessing import Process

from SmartWaveAPI import SmartWave
import SmartWaveAPI
from imu_conf_lib import *
from sys import argv, exit
import sys
from inspect import currentframe, getframeinfo



try:
    com = argv[1]
except:
    com = None

matplotlib.use('TkAgg')

# If set to true, include the io-expander for visualization
IO_EXPANDER = True

# If set to true, the script generates and plays a sine wave according to the IMU sensor data
PLAY_SOUND = False 

# If set to true, the script prints out falls that occur
FALL_DETECTION = True

# Color scheme for plotting
sem_grey = '#323c40'
sem_white = '#f2f2f2'
sem_blue = '#43788c'
sem_dark_blue = '#2b4c59'
sem_light_grey = '#6d848c'


def read_isr_i2c(sw):
    isr = sw.readFPGARegister(0x840a0)
    enable = sw.readFPGARegister(0x84000)
    cmd_err = isr & 0x1
    data_err = (isr >> 1) & 0x1
    print(f"IMU Enable: {enable}")
    print(f"IMU ISR Command Frame Error field: {cmd_err}")
    print(f"IMU ISR Data Frame Error field: {data_err}\n")
    isr = sw.readFPGARegister(0x841a0)
    enable = sw.readFPGARegister(0x84100)
    cmd_err = isr & 0x1
    data_err = (isr >> 1) & 0x1
    print(f"IO_EXP Enable: {enable}")
    print(f"IO_EXP ISR Command Frame Error field: {cmd_err}")
    print(f"IO_EXP ISR Data Frame Error field: {data_err}\n")
    


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
    # ctrl1_xl = i2c.readRegister(i2c_addr, 0x10.to_bytes(1, 'big'), 1, timeout=None)
    # print(f"Accelerometer control register value: {ctrl1_xl[0]:08b}")


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
    # ctrl2_g = i2c.readRegister(i2c_addr, 0x11.to_bytes(1, 'big'), 1, timeout=None)
    # print(f"Gyroscope control register value: {ctrl2_g[0]:08b}")

def disable_i2c0(sw):
    #disable i2c0 driver
    sw.writeFPGARegister(0x84000, 0)
    #disable stim mem 0
    sw.writeFPGARegister(0x60000, 0)

def enable_i2c0(sw):
    sw.writeFPGARegister(0x84000, 1)
    sw.writeFPGARegister(0x60000, 1)

def disable_i2c1(sw):
    sw.writeFPGARegister(0x84100, 0)
    sw.writeFPGARegister(0x60100, 0)

def enable_i2c1(sw):
    sw.writeFPGARegister(0x84100, 1)
    sw.writeFPGARegister(0x60100, 1)


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


def generate_sound(frequency: int = 440, duration: float = 1, harmonics: int = 0, pitch_shift: int = 0):
    """
    Generate a sinusoidal signal from the user defined parameters
    :param frequency: Fundamental frequency of the sine wave
    :param duration: Duration of the signal in seconds
    :param harmonics: Number of harmonics to add to the signal
    :param pitch_shift: Pitch shift in octaves (positive for increase, negative for decrease)
    :return: None
    """
    # Bluetooth speaker sample rate ~48kHz
    fs = 20000    # Sampling frequency   44100
    # Scale the period of the signal according to the pitch shift
    if pitch_shift < 0:
        duration = duration / (2 ** (pitch_shift * -1))

    if pitch_shift > 0:
        duration = duration * 2 ** pitch_shift

    # Generate the fundamental sine wave
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    signal_wave = np.sin(2 * np.pi * frequency * t)

    for harmonic in range(2, harmonics + 1):
        if harmonic % 2 == 1:
            signal_wave += np.sin(2 * np.pi * frequency * harmonic * t) / harmonic

    if pitch_shift != 0:
        resample_factor = 2 ** -pitch_shift
        signal_wave = signal.resample(signal_wave, int(len(signal_wave) * resample_factor))

    return signal_wave

def reset_soc(sw):
    SOC_CTRL_REG = 0x48010
    # assert reset SoC and block instruction fetch
    sw.writeFPGARegister(SOC_CTRL_REG, 0b00)
    sw.writeFPGARegister(SOC_CTRL_REG, 0b11)


def fall_detected(sample):

    # debounce the fall detection
    if(default_timer() - fall_detected.start_fall > 0.3):
        if((sample[3] > 0x0700 and sample[3] < 0x3f00) or
           (sample[4] > 0x0700 and sample[4] < 0x3f00) or
           (sample[5] > 0x0700 and sample[5] < 0x3f00)):
            fall_detected.start_fall = default_timer()
            return True
    
    return False
fall_detected.start_fall = -0.3

def load_firmware(sw):

    path = "../../IMU_sensor_demo/python/firmware.mem"
    try:
        firmware = open(path, "r")
    except:
        exit("Firmware not found...")


    SOC_CTRL_REG = 0x48010      # TODO: import these values from a header file

    sw.writeFPGARegister(SOC_CTRL_REG, 0b00)
    addr = 0xC2000

    # write the firmware into IRAM
    for line in firmware:
        # write a line from the firmware to the RAM
        l = line.strip()
        h = int(l, 16)
        sw.writeFPGARegister(addr, h)

        addr = addr + 0x1;
    
    # reset SoC and enable instruction fetch
    sw.writeFPGARegister(SOC_CTRL_REG, 0b11)

def read_button(sw):
    button = (sw.readFPGARegister(0x46038) >> 3) & 0x01
    button = int(not button)
    print(f'button: {button}')

def read_output_sel0(sw):
    print("Pinmux Output Sel 0 : \n0x%x\n" % sw.readFPGARegister(0x46000))


# Import semify logo for plotting
file = "../../IMU_sensor_demo/python/semify_logo.png"
img = Image.open(file)
resize = img.resize((np.array(img.size) / 19).astype(int))  # / 17


sensor_emulation = False
imu_addr = 0x6a  # Default I2C address
emulated_addr = 0x7a # emulated I2C address
i2c_addr = emulated_addr if sensor_emulation else imu_addr
edge = False
previous = False


def main():
    """
    The main function handles the connection to SmartWave and the ASM330LHHXG1 IMU Sensor. It configures the
    accelerometer and gyroscope, sets default plotting parameters, and reads the angular and linear rate registers
    for data visualization.
    :return: None
    """

    try:
        with SmartWave().connect(reset=False, port_name=com) as sw: 

            # add tags on display for i2ct and button
            sw.createGPIO("A4", "BTN")
            SmartWaveAPI.configitems.GPIO.color = "#1E88E5"
            sw.createGPIO("A1", "EMU SDA")
            sw.createGPIO("A7", "EMU SCL")

            #TODO
            #configure pinmux to output/input TSDA and TSCL on pins A1 and A7
            enable_i2c0(sw)
            i2c_imu = sw.createI2CConfig(sda_pin_name="A2", scl_pin_name="A3", sda_display_name="IMU SDA", scl_display_name="IMU SCL", clock_speed=int(400e3))

            try:
                imu_id = i2c_imu.readRegister(i2c_addr, 0x0f.to_bytes(1, 'big'), 1)
                disable_i2c0(sw)
            except:
                try:
                    imu_id = i2c_imu.readRegister(i2c_addr, 0x0f.to_bytes(1, 'big'), 1)
                    disable_i2c0(sw)
                except:
                    SmartWave.disconnect(sw)
                    exit("Connection to SmartWave was successful, but unable to establish connection with IMU device on address 0x6a...")

            try:
                if IO_EXPANDER:
                    i2c_io_exp_addr = 0x20

                    enable_i2c1(sw)
                    i2c_io_exp = sw.createI2CConfig(sda_pin_name="A10", scl_pin_name="A9", sda_display_name="IO SDA", scl_display_name="IO SCL", clock_speed=int(400e3))
                    i2c_io_exp.write(i2c_io_exp_addr, [0x00, 0x00])
                    i2c_data = i2c_io_exp.read(i2c_io_exp_addr, 2)
                    disable_i2c1(sw)

                    if i2c_data.ack_device_id is False:
                        print("Error, couldn't connect to IO expander!")
                    elif i2c_data.ack_device_id is True:
                        print("Successfully connected to IO expander")
            except Exception as e:
                print('unable to connect to IO expander...')
                print(e)

            # Check if connection to the target device was successful
            if (imu_id[0] == 0xff) or (imu_id[0] == 0x00):
                exit("Couldn't reach device. Terminating code.")
            else:
                print(f"Connection was successful. Device ID: {imu_id[0]:#0x}")

            try:
                # Configure the ASM330LHHXG1 IMU
                enable_i2c0(sw)

                axl_conf(i2c_imu, imu_addr)
                gyro_conf(i2c_imu, imu_addr)
                # axl_conf(i2c_imu, emulated_addr)
                # gyro_conf(i2c_imu, emulated_addr)

                print('IMU successfully configured')
                disable_i2c0(sw)
            except:
                print('error in configuring IMU')
                exit()

            # Default settings for plotting
            plt.rcParams["figure.facecolor"] = sem_grey
            plt.rcParams["figure.figsize"] = [9.50, 7.50]
            plt.rcParams["figure.autolayout"] = True
            x_len = 200

            global fig
            fig = plt.figure()
            gs = fig.add_gridspec(3, 2)
            ax1 = fig.add_subplot(gs[0, 0])
            ax2 = fig.add_subplot(gs[1, 0])
            ax3 = fig.add_subplot(gs[2, 0])
            ax4 = fig.add_subplot(gs[0, 1])
            ax5 = fig.add_subplot(gs[1, 1])
            ax6 = fig.add_subplot(gs[2, 1])

            fig.suptitle("Inertial Measurement Unit\nData Visualization", color=sem_white)

            xmax = fig.bbox.xmax
            ymax = fig.bbox.ymax

            # plt.figimage(resize, xo=int(fig.bbox.xmax // 2) + 650, yo=int(fig.bbox.ymax) + 220)
            fig.text(x=0.05, y=0.95 , s='Sensor Emulation: ', color=sem_white, fontsize='large')
            fig.text(x=0.213, y=0.949 , s='OFF', color='#dd0000', fontsize='x-large', fontweight='extra bold')
            fig.text(x=0.213, y=0.949 , s='ON', color='#00dd00', fontsize='x-large', fontweight='extra bold', visible=False)

            ax1.set_title("X - axis", color=sem_white)
            ax2.set_title("Y - axis", color=sem_white)
            ax3.set_title("Z - axis", color=sem_white)

            ax4.set_title("Pitch", color=sem_white)
            ax5.set_title("Roll", color=sem_white)
            ax6.set_title("Yaw", color=sem_white)

            for ax in [ax1, ax2, ax3]:
                ax.set_ylim(-20, 20)
                ax.set_ylabel("Acceleration (m/s^2)", color=sem_white)
                ax.set_xlabel("Samples", color=sem_white)
                ax.tick_params(labelcolor=sem_white)
                ax.grid()

            for ax in [ax4, ax5, ax6]:
                ax.set_ylim(-300, 300)
                ax.set_ylabel("Rate of Change", color=sem_white)
                ax.yaxis.set_label_position('right')
                ax.yaxis.tick_right()
                ax.set_xlabel("Samples", color=sem_white)
                ax.tick_params(labelcolor=sem_white)
                ax.grid()

            xs = list(range(0, 200))

            axl_x = [0] * x_len
            axl_y = [0] * x_len
            axl_z = [0] * x_len
            line1, = ax1.plot(xs, axl_x, lw=2, color=sem_dark_blue)
            line2, = ax2.plot(xs, axl_y, lw=2, color=sem_blue)
            line3, = ax3.plot(xs, axl_z, lw=2, color=sem_light_grey)

            gyro_x = [0] * x_len
            gyro_y = [0] * x_len
            gyro_z = [0] * x_len
            line4, = ax4.plot(xs, gyro_x, lw=2, color=sem_dark_blue)
            line5, = ax5.plot(xs, gyro_y, lw=2, color=sem_blue)
            line6, = ax6.plot(xs, gyro_z, lw=2, color=sem_light_grey)

            line = [line1, line2, line3, line4, line5, line6]
            ys = [axl_x, axl_y, axl_z, gyro_x, gyro_y, gyro_z]

            def animate(i, ys):
                """
                This function is used to animate the plotting, which enables displaying real-time data.
                :param i: Placeholder
                :param ys: List that contains data for plotting the linear rate of change
                :return: Data for plotting
                """
                try:
                    global sensor_emulation
                    global i2c_addr
                    global imu_addr
                    global emulated_addr
                    global edge
                    global previous
                    global fig

                    # fig.text(0, 0, "Sensor Emulation", color='red', fontsize='x-large')

                    # detect switch between sensor emulation ond pysical imu
                    if(keyboard.is_pressed('space')):
                        if(not previous):
                            edge = True
                        else:
                            edge = False
                        if(edge):
                            sensor_emulation = not sensor_emulation
                            i2c_addr = emulated_addr if sensor_emulation else imu_addr

                            if(sensor_emulation):
                                print('emulation on')
                                for t in fig.texts:
                                    if(t.get_text() == 'ON'):
                                        t.set_visible(True)
                                    if(t.get_text() == 'OFF'):
                                        t.set_visible(False)

                            if(not sensor_emulation):
                                print('emulation off')
                                for t in fig.texts:
                                    if(t.get_text() == 'ON'):
                                        t.set_visible(False)
                                    if(t.get_text() == 'OFF'):
                                        t.set_visible(True)
                                
                            fig.show()

                        previous = True
                    else:
                        previous = False
                        edge = False
                    

                    enable_i2c0(sw)
                    pitch_lsb = i2c_imu.readRegister(i2c_addr, 0x22.to_bytes(1, 'big'), 1)
                    pitch_msb = i2c_imu.readRegister(i2c_addr, 0x23.to_bytes(1, 'big'), 1)
                    pitch = (pitch_msb[0] << 8) + pitch_lsb[0]
                    tc_pitch = twos_comp(pitch)

                    roll_lsb = i2c_imu.readRegister(i2c_addr, 0x24.to_bytes(1, 'big'), 1)
                    roll_msb = i2c_imu.readRegister(i2c_addr, 0x25.to_bytes(1, 'big'), 1)
                    roll = (roll_msb[0] << 8) + roll_lsb[0]
                    tc_roll = twos_comp(roll)

                    yaw_lsb = i2c_imu.readRegister(i2c_addr, 0x26.to_bytes(1, 'big'), 1)
                    yaw_msb = i2c_imu.readRegister(i2c_addr, 0x27.to_bytes(1, 'big'), 1)
                    yaw = (yaw_msb[0] << 8) + yaw_lsb[0]
                    tc_yaw = twos_comp(yaw)

                    # Angular rate conversion - returns the rate of change
                    pitch_res = np.round((tc_pitch * gyro_sense['4000_dps']), 3)  # * (np.pi / 180)), 3)
                    roll_res = np.round((tc_roll * gyro_sense['4000_dps']), 3)  # * (np.pi / 180)), 3)
                    yaw_res = np.round((tc_yaw * gyro_sense['4000_dps']), 3)  # * (np.pi / 180)), 3)
                    # print(f"Pitch: {tc_pitch:.3f}     Roll: {tc_roll:.3f}    Yaw: {tc_yaw:.3f}")
                    # print(f"Pitch: {pitch_res:.3f} degrees    Roll: {roll_res:.3f} degrees   Yaw: {yaw_res:.3f} degrees\n")

                    ys[3].append(pitch_res)
                    ys[3] = ys[3][-x_len:]
                    line[3].set_ydata(ys[3])

                    ys[4].append(roll_res)
                    ys[4] = ys[4][-x_len:]
                    line[4].set_ydata(ys[4])

                    ys[5].append(yaw_res)
                    ys[5] = ys[5][-x_len:]
                    line[5].set_ydata(ys[5])

                    # Linear acceleration sensor
                    x_axis_lsb = i2c_imu.readRegister(i2c_addr, 0x28.to_bytes(1, 'big'), 1)
                    x_axis_msb = i2c_imu.readRegister(i2c_addr, 0x29.to_bytes(1, 'big'), 1)
                    x_axis = (x_axis_msb[0] << 8) + x_axis_lsb[0]
                    tc_x_axis = twos_comp(x_axis, 16)

                    y_axis_lsb = i2c_imu.readRegister(i2c_addr, 0x2A.to_bytes(1, 'big'), 1)
                    y_axis_msb = i2c_imu.readRegister(i2c_addr, 0x2B.to_bytes(1, 'big'), 1)
                    y_axis = (y_axis_msb[0] << 8) + y_axis_lsb[0]
                    tc_y_axis = twos_comp(y_axis, 16)

                    z_axis_lsb = i2c_imu.readRegister(i2c_addr, 0x2C.to_bytes(1, 'big'), 1)
                    z_axis_msb = i2c_imu.readRegister(i2c_addr, 0x2D.to_bytes(1, 'big'), 1)
                    z_axis = (z_axis_msb[0] << 8) + z_axis_lsb[0]
                    tc_z_axis = twos_comp(z_axis, 16)

                    # Linear acceleration conversion
                    # convert from g to m/s^2 1 g-unit = 9.80665
                    x_res = tc_x_axis * axl_sens['2g'] * 9.80665
                    y_res = tc_y_axis * axl_sens['2g'] * 9.80665
                    z_res = tc_z_axis * axl_sens['2g'] * 9.80665
                    # print(f"X_adc: {pitch:.3f}     Y_adc: {roll:.3f}    Z_adc:: {yaw:.3f}")
                    # print(f"X_a: {x_res:.3f} m/s^2    Y_a: {y_res:.3f} m/s^2   Z_a: {z_res:.3f} m/s^2\n")

                    sample = ['0x%04x' % x_axis, '0x%04x' % y_axis, '0x%04x' % z_axis, '0x%04x' % pitch, '0x%04x' % roll, '0x%04x' % yaw]
                    sample_int = [x_axis, y_axis, z_axis, pitch, roll, yaw]

                    if(fall_detected(sample_int) and FALL_DETECTION):
                            print("Fall Detected!")

                    # print(f"pitch: {'0x%04x' % pitch}    roll: {'0x%04x' % roll}    yaw: {'0x%04x' % yaw}")


                    ys[0].append(y_res)
                    ys[0] = ys[0][-x_len:]
                    line[0].set_ydata(ys[0])

                    ys[1].append(x_res)
                    ys[1] = ys[1][-x_len:]
                    line[1].set_ydata(ys[1])

                    ys[2].append(z_res)
                    ys[2] = ys[2][-x_len:]
                    line[2].set_ydata(ys[2])

                    disable_i2c0(sw)
    
                    if IO_EXPANDER:
                        try:
                            enable_i2c1(sw)
                            if(sensor_emulation):
                                io_led_toggle(i2c_io_exp, i2c_io_exp_addr, x_res, y_res)
                            else:
                                io_led_toggle(i2c_io_exp, i2c_io_exp_addr, y_res, x_res)
                            disable_i2c1(sw) 
                        except:
                            print('unable to toggle IO leds')
                            exit()

                    if PLAY_SOUND:
                        frequency = 220
                        duration = 0.1

                        harmonics, pitch_shift = sound_modulation(y_res, x_res)

                        signal_wave = generate_sound(frequency, duration, harmonics, pitch_shift)

                        p = pyaudio.PyAudio()
                        stream = p.open(format=pyaudio.paFloat32,
                                        channels=1,
                                        rate=20000,
                                        output=True)
                        stream.write(signal_wave.astype(np.float32).tobytes())
                        stream.stop_stream()

                    return line

                except Exception as e:
                    print("Error on line {}".format(sys.exc_info()[-1].tb_lineno))
                    print("Connection to IMU unit has been interrupted. Terminating Program.")
                    print(e)
                    SmartWave.disconnect(sw)
                    exit()

            anim = animation.FuncAnimation(fig, animate,
                                        fargs=(ys,),
                                        interval=1,
                                        blit=True,
                                        cache_frame_data=False)

            plt.figimage(resize, origin='upper')
            plt.show()
    except Exception as e:
        print(e)
        print("Unable to connect to SmartWave device, please check connection...")

if __name__ == "__main__":
    process = Process(target=main)
    process.start()
    while process.is_alive():
        if keyboard.is_pressed('q'):
            print("Terminating code.")
            process.terminate()
            try:
                time.sleep(0.25)
                SmartWave.disconnect()
            except:
                pass
            break
