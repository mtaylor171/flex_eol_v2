from ctypes import *
import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import csv
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import datetime
from datetime import timedelta
import time
import sys
import random
import RPi.GPIO as GPIO
import os
import pigpio
import calculate_rms
import math

ACTIVE_CHANNELS = 8

kDt = 0.5
kAlpha = 0.01
kBeta = 0.0001

def get_us():
    now = time.perf_counter()
    return now

# returns the elapsed time by subtracting the timestamp provided by the current time n
def get_elapsed_us(timestamp):
    temp = get_us()
    return (temp - timestamp)

class MotorController(object):
    SO_FILE = os.path.dirname(os.path.realpath(__file__)) + "/motor_spi_lib.so"
    C_FUNCTIONS = CDLL(SO_FILE)
    
    def __init__(self, file, pwm_target, motor_duration, mode = GPIO.BOARD, freq = 25000, warnings = False):
        self.pwm_pin = 19
        self.motor_pin = 15
        self.pi = pigpio.pi()
        self.INITIAL_US = get_us()
        self.file = file
        GPIO.setwarnings(warnings)
        GPIO.setmode(mode)
        GPIO.setup(self.motor_pin, GPIO.OUT)
        
        ## Default values
        self.pwm_target = pwm_target
        self.motor_duration = motor_duration
        self.pwm_current = 19
        self.position_hold_time = 0
        self.position_counter = 0
        self.data = [[],[],[],[],[],[],[],[],[]]
        self.last_position = 0
        self.freq_count = [[],[]]
        self.rms_data_full = []
        self.csv_data = []
        self.current_rev_time = 0
        self.last_rev_time = 0
        self.master_pos_counter = 0
        self.last_current_index = 2
        self.rms_timestamp = 0
        self.rms_avg = [0,0,0,0,0]
        self.rms_counter = 0
        self.freq = 0
        self.timestamp_steady_state = 0

        self.phaseA_rms_current_1sec = []
        self.phaseB_rms_current_1sec = []
        self.phaseC_rms_current_1sec = []

        self.kX1 = 0.0
        self.kV1 = 0.0
        self.x = []
        self.v = []
        self.r = []


    def initialize(self):
        print("\n*****************************\n")
        print("Initializing Motor Board")
        print("\n*****************************\n")
        msg = ""
        self.pi.hardware_PWM(self.pwm_pin, 0, 0)

        _self_check = self.C_FUNCTIONS.adc_setlow()
        if not _self_check:
            pass

        GPIO.output(self.motor_pin, 1)
        self.INITIAL_US = get_us()
        _self_check = self.C_FUNCTIONS.initialize_motor()

        if not _self_check:
            #print("Motor Initialized Successfully")
            pass

        else:
            ## TODO Raise exception here
            msg = "ERROR: Could not communicate with motor board. Please disconnect motor."
            return 0, msg
        '''
        if input("Would you like to view the registers? (y/n): ").lower() == 'y':
            self._read_registers()

            if(input("\nAre Registers correct? (y/n): ").lower() != 'y'):
                msg = "Registers Not Selected to be Correct."
                return 0, msg
        '''
        if not self.C_FUNCTIONS.initialize_adc():
            #print("ADC Initialized Successfully")
            pass
        else:
            msg = "ERROR: ADC Initialize Failed. Please Disconnect motor."
            return 0, msg

        return 1, "Initialization complete!"

    def analog_in_initial_send(self):
        self.C_FUNCTIONS.getAnalogInAll_InitialSend()

    # Increases PWM control duty cycle by 1%
    # Gets called by run_main until preferred duty cycle is reached
    def pwm_control(self):
        if(self.pwm_current < self.pwm_target):
            self.pwm_current += 1
        self.pi.hardware_PWM(self.pwm_pin, 25000, self.pwm_current * 10000)

    def bcm2835_init_spi(self):
        self.C_FUNCTIONS.AD5592_Init()

    def bcm2835_motor_ping(self):
        GPIO.output(self.motor_pin, 1)
        return self.C_FUNCTIONS.motor_ping()

        if not self.C_FUNCTIONS.initialize_adc():
            pass

    def get_analog_data(self):
        return self.C_FUNCTIONS.getAnalogInAll_Receive()
    
    def analog_terminate(self):
        self.C_FUNCTIONS.getAnalogInAll_Terminate()

    def health_check(self, temp_data):
        code = [0,0,0]
        self.csv_data = []
        for i in range(1,4):            # Turning Hall sensor channel data into a 3-digit position code
            if(temp_data[i] > 1500):    # Set a threshold of 1650mV for the hall pulse
                code[i-1] = 1
            else:
                code[i-1] = 0
        position = self._find_positions(code)   # Convert code into a position (1-6)
        if(self.last_position != position):     # Check if position is different from the last recorded position
            if(self.last_position != 0):
                self.master_pos_counter += 1
                self.position_counter += 1 
                if((self.position_counter % 30) == 0):
                    self.current_rev_time = get_us()
                    self.freq = self._get_rpm(self.current_rev_time, self.last_rev_time)
                    self.last_rev_time = self.current_rev_time
                    #self.running_filter(freq)
                #if(self.position_counter == 270):
                    #self._calculate_rms(self.last_current_index, (len(self.data[0]) - 1))
                    #self.last_current_index = (len(self.data[0]) - 1)
                    #self.csv_data.insert(1, round(self.freq, 1))
                    #writer = csv.writer(self.file)
                    #writer.writerow(self.csv_data)
                    self.position_counter = 0
                    #print('\033c')
                    #print("Time: {} ".format(round(get_elapsed_us(self.INITIAL_US), 1)) + "PWM: {} ".format(self.pwm_current) + "RPM: {} ".format(round(self.freq, 1)) + "Current: {}".format(self.csv_data[2:]))
                    #print('\033c')
                    #print("RPM: {} ".format(freq))
                else:
                    rms_val = 0
                #print("Elapsed: {}, ".format(get_elapsed_us(self.INITIAL_US)) + "Position: {}, ".format(position) + "Frequency: {} ".format(round(freq, 2)) + "Filtered freq: {} ".format(x[-1]) +"PWM: {} ".format(self.pwm_current) + "Freq/PWM = {} ".format(reluctance) + "RMS Current: {}".format(rms_val))
            else:
                pass
                #msg = "INCORRECT POSITION RECORDED"
                #return 0, msg
            self.position_hold_time = get_us()
            self.last_position = position
        else:
            if get_elapsed_us(self.position_hold_time) > 1:
                msg = "STALL DETECTED"
                return 0, msg
        
        if(len(self.data[0]) > 2) and (temp_data[0] - self.data[0][self.last_current_index - 1] >= 1000000):
            self._calculate_rms(self.last_current_index - 1, (len(self.data[0]) - 1))
            self.last_current_index = (len(self.data[0]))
            self.csv_data.insert(1, round(self.freq, 1))
            print('\033c')
            print("Time: {} ".format(round(get_elapsed_us(self.INITIAL_US), 1)) + "PWM: {} ".format(self.pwm_current) + "RPM: {} ".format(round(self.freq, 1)) + "Current: {}".format(self.csv_data[2:]))

            writer = csv.writer(self.file)
            writer.writerow(self.csv_data)

            
        return 1, "All Good!"

    def running_filter(self, data):
        x_k = self.kX1 + kDt * self.kV1
        r_k = data - x_k
        x_k = x_k + kAlpha * r_k
        v_k = self.kV1 + (kBeta/kDt) * r_k

        self.kX1 = x_k
        self.kV1 = v_k

        self.x.append(x_k)
        self.v.append(v_k)
        self.r.append(r_k)        

    def rampdown(self):
        print("Starting rampdown...")
        for duty in range(self.pwm_current, 0, -1):
            self.pi.hardware_PWM(self.pwm_pin, 25000, duty * 10000)
            print("RAMPDOWN -- PWM: {}".format(duty))
            time.sleep(0.2)
        self.pi.hardware_PWM(self.pwm_pin, 0, 0)
        
    def shutdown(self):
        print("Starting Shutdown")
        for duty in range(self.pwm_current, 0, -1):
            self.pi.hardware_PWM(self.pwm_pin, 25000, duty * 10000)
            print("SHUTDOWN -- PWM: {}".format(duty))
            time.sleep(0.05)
        self.pi.hardware_PWM(self.pwm_pin, 0, 0)
        GPIO.output(self.motor_pin, 0)

    def killall(self):
        self.pi.hardware_PWM(self.pwm_pin, 0, 0)
        GPIO.output(self.motor_pin, 0)
        #self.pi.close()

    def motor_results(self, resp, msg, rms):
        print("\n\n-----------------------------\n")
        print("-----------------------------\n")
        if not resp:
            print("MOTOR FAILED\n")
            print(msg)
        else:
            print("MOTOR PASSED\n")
        print("\n\n-----------------------------\n")
        print("-----------------------------\n")

    def _calculate_rms(self, c_start, c_finish):
        self.csv_data.append(self.data[0][c_finish])
        for i in range(4, 7):
            temp_sum = np.int64(0)
            temp_rms = np.float64(0.0)
            for j in range(c_start, c_finish+1):
                temp_sum += (2 * (((self.data[i][j])/1000)**2) * ((self.data[0][j] - self.data[0][j-1])))
                #print(temp_sum)
            temp_rms = temp_sum/((self.data[0][c_finish] - self.data[0][c_start]))
            temp_rms = round((math.sqrt(temp_rms)), 3)
            self.csv_data.append(temp_rms)
        
    def _calculate_rms_full(self):

        for i in range(4, 7):
            temp_sum = np.int64(0)
            temp_rms = np.float64(0.0)
            for j in range(self.data[0].index(self.timestamp_steady_state), len(self.data[0])):
                temp_sum += (2 * ((self.data[i][j])**2) * ((self.data[0][j] - self.data[0][j-1]) / 1000))
            temp_rms = temp_sum/((self.data[0][len(self.data[0]) - 1] - self.data[0][0]) / 1000)
            self.rms_data_full.append(round((math.sqrt(temp_rms)), 3))
        return self.rms_data_full

    def _read_registers(self):
    # Reads all registers on DRV8343 and prints them
        for i in range(19):
            reg_data = self.C_FUNCTIONS.motor_register_read(i)
            print('Register {}:'.format(i) + ' {}'.format(hex(reg_data)));
            print('\n')

    def _find_positions(self, code):
    # Converts the hall sensor pulse data into a position (1-6)
    # If the hall sensor pulses do not align with one of these positions, a zero is returned at which there will a flag raised
        if code == [1, 0, 1]:
            return 1
        elif code == [0, 0, 1]:
            return 2
        elif code == [0, 1, 1]:
            return 3
        elif code == [0, 1, 0]:
            return 4
        elif code == [1, 1, 0]:
            return 5
        elif code == [1, 0, 0]:
            return 6
        else:
            return 7
    
    def _get_rpm(self, current_rev_time, last_rev_time):

        freq = 60*( 1/((current_rev_time - last_rev_time)*3) )
        self.freq_count[0].append(get_elapsed_us(self.INITIAL_US))
        self.freq_count[1].append(freq)
        return freq

    def _motor_reluctance(self, freq):
        #return freq/self.pwm_current
        return 0
        
    def _revolution_rms(self):
        #TODO: Implement Function here
        return 0

def graph_freq(MC_1, MC_2):

    plt.xlabel('Time (ms)')

    plt.plot(MC_1.freq_count[0], MC_1.freq_count[1])
    plt.plot(MC_2.freq_count[0], MC_2.freq_count[1])
    
    plt.legend([f"TEST1 - PWM target: {MC_1.pwm_target}%", f"TEST2 - PWM target: {MC_2.pwm_target}%"])
    plt.show()

def start_sequence():
    print('\033c')
    print("*****************************")
    print(f"NURO MOTOR TESTING - {datetime.datetime.now().replace(microsecond=0)}")
    print("*****************************\n")

    MC_start = MotorController(None, 0, 0)

    MC_start.bcm2835_init_spi()

    print("Waiting on motor board to power up...")
    print("(NOTE: Hold CTRL + 'C' to exit program)\n")

    try:
        if not MC_start.C_FUNCTIONS.adc_setlow():
            pass
        while(MC_start.bcm2835_motor_ping()):
            pass
        if not MC_start.C_FUNCTIONS.initialize_adc():
            pass
        print('\033c')
        print("*****************************")
        print("Motor Board Connected!")
        print("*****************************")
        
        return 1
        

    except KeyboardInterrupt:
        end_sequence(MC_start)

        return 0

def end_sequence(MC):
    MC.killall()

def run_motor(MC, file):
    temp_data = np.uint32([0,0,0,0,0,0,0,0,0])
    adc_reading = 0x0
    index = 0x0
    pwm_counter = 0
    
    resp, msg = MC.initialize()
    if not resp:
        end_sequence(MC)
        return -1, msg
    
    MC.analog_in_initial_send()

    MC.position_hold_time = MC.revolution_hold_time = get_us()

    while(1):
        if(MC.pwm_current < MC.pwm_target):                              # Ramps up PWM
            if( (pwm_counter == 0) or ((pwm_counter % 1000) == 0) ):
                MC.pwm_control()
                if(len(MC.data[0]) > 1):
                    MC.timestamp_steady_state = MC.data[0][-1]
            pwm_counter += 1

        for i in range(0, ACTIVE_CHANNELS):
            data_16bit = MC.get_analog_data() 
            adc_reading, index = data_process(data_16bit)
            temp_data[index+1] = adc_reading

        for i in range(1, 9): 
            MC.data[i].append(temp_data[i])

        temp_data[0] = int(round(get_elapsed_us(MC.INITIAL_US), 6) * 1000000)
        MC.data[0].append(temp_data[0])

        writer = csv.writer(file)
        writer.writerow(temp_data)

        try:
            resp, msg = MC.health_check(temp_data)
            if not resp:
                MC.analog_terminate()
                MC.shutdown()
                return -1, msg
            if(temp_data[0] >= MC.motor_duration * 1000000):
                MC.analog_terminate()
                MC.rampdown()
                msg = "Motor duration reached: {}".format(temp_data[0])
                return 1, msg
        except KeyboardInterrupt:

            MC.analog_terminate()
            MC.rampdown()
            msg = "----Keyboard Interrupt by user----"
            return -1, msg

        finally:
            pass

def data_process(data):
    index = ((data >> 12) & 0x7)
    data_converted = int(data & 0xFFF) * (5000/4095)
    if index in range(0,3): # Channels 0-2 are hall sensors - use voltage translation
        adc_reading = data_converted
    elif index in range(3,6): # Channes 3-5 are current sensors - use current translation
        #adc_reading = (3000 - data_converted)
        if data_converted >= 3000:
            adc_reading = 0
        else:
            adc_reading = round((10 * (3000 - data_converted)), 2)
    elif index in range(6,8):
        adc_reading = ((data_converted - 409.5) * 0.7535795) + 25
        #adc_reading = int(((data_converted - 409.5) * 0.7535795) + 25)
    return adc_reading, index
    #return data_converted, index

def message_display(msg, desired_answer):
    while(1):
        if input(msg).lower() == desired_answer:
            return 1
        else:
            print('\033c')
            print("*****************************")
            print("Incorrect character entered.")
            print("*****************************")
            return 0

def delete_files(FILE_OUTPUT_NAME):
    if(os.path.exists("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode1_fulldata")):
        os.remove("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode1_fulldata")
    if(os.path.exists("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode2_fulldata")):
        os.remove("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode2_fulldata")

def run_main():
        
    FILE_OUTPUT_NAME = str(datetime.datetime.now().replace(microsecond=0))
    file1 = open("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode1_rms_rpm", 'w', newline='')
    file2 = open("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode2_rms_rpm", 'w', newline='')

    file1_full = open("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode1_fulldata", 'w', newline='')
    file2_full = open("/home/pi/Documents/___FLEX_MOTOR_DATA_FOLDER/" + FILE_OUTPUT_NAME + " mode2_fulldata", 'w', newline='')

    MC_1 = MotorController(file1, 25, 5)
    
    resp, msg = MC_1.initialize()
    if not resp:
        end_sequence(MC_1)
        print(msg)
        return -1
    MC_2 = MotorController(file2, 45, 5)
    
    print('\033c')
    print("*****************************")
    print(f"FILES FOR THIS TEST WILL BE SAVED WITH THE TIMESTAMP: {FILE_OUTPUT_NAME}")

    print(f"\nMode 1 settings: {MC_1.pwm_target}%, {MC_1.motor_duration}secs")
    print(f"Mode 2 settings: {MC_2.pwm_target}%, {MC_2.motor_duration}secs\n")

    #print('\033c')
    
    try:
        while(message_display("To begin testing, press 'y' and ENTER: ", 'y') != 1):
            pass
        print('\033c')
        print("*****************************")
        print("----Testing Mode 1----")

        resp1, msg1 = run_motor(MC_1, file1_full)
        print(msg1)
        if resp1 < 0:
            #print('\033c')
            print(msg1)
            while(message_display("\nType 'c' and ENTER to continue: ", 'c') != 1):
                pass
            print('\033c')
            print("\nRestarting test program...")
            delete_files(FILE_OUTPUT_NAME)
            time.sleep(3)
            return -1

        print('\033c')
        print("*****************************\n")
        print("----Testing Mode 2----")
        time.sleep(2)
        resp2, msg2 = run_motor(MC_2, file2_full)
        print(msg2)
        #end_sequence(MC_2)
        if resp2 < 0:
            #print('\033c')
            print(msg2)
            print("***Please disconnect motor and restart motor board***")
            while(message_display("\nType 'c' and ENTER once motor disconnected: ", 'c') != 1):
                pass
            print('\033c')
            print("Restarting test program...")
            delete_files(FILE_OUTPUT_NAME)
            time.sleep(3)
            return -1

        #print('\033c')
        print("*****************************\n")
        print("\nGenerating results. This may take up to a minute...\n")
        rms1, rms2 = calculate_rms.main(FILE_OUTPUT_NAME + " mode1_fulldata", FILE_OUTPUT_NAME + " mode2_fulldata", MC_1.data[0].index(MC_1.timestamp_steady_state), MC_2.data[0].index(MC_2.timestamp_steady_state))
        #print(f"Phase RMS for mode1 [A, B, C]: {rms1}")
        #print(f"Phase RMS for mode2 [A, B, C]: {rms2}")

        MC_1.motor_results(resp1, msg1, rms1)
        MC_2.motor_results(resp1, msg1, rms2)

        print('\033c')
        print("Please disconnect motor!\n")
        while( message_display("Press 'c' and ENTER to continue to next motor, or CTRL + 'C' to exit program: ", 'c') != 1):
            pass
        delete_files(FILE_OUTPUT_NAME)
        time.sleep(1)
        return 1
    except KeyboardInterrupt:
        delete_files(FILE_OUTPUT_NAME)
        end_sequence(MC_1)
        end_sequence(MC_2)
        return 0

if __name__ == "__main__":
    while(1):
        if start_sequence() == 0:
            sys.exit()

        while(1):
            state = run_main()

            if state == 0 :
                print('\033c')
                print("*****************************")
                print("This program will be shutting down in 3 seconds")
                print("*****************************")
                time.sleep(3)
                sys.exit()

            elif state == -1:
                break

            else:
                pass


