import time
import pigpio
import sys

class reader:

    def __init__(self, pi, gpio, pwm, pulses_per_rev = 1.0, weighting = 0.0, min_RPM = 5.0):

        self.pi = pi
        self.gpio = gpio
        self.pwm = pwm
        self.pulses_per_rev = pulses_per_rev

        if min_RPM > 1000.0:
            min_RPM = 1000.0
        elif min_RPM < 1.0:
            min_RPM = 1.0

        self.min_RPM = min_RPM
        self._watchdog = 200 	# milliseconds

        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99

        self._new = 1.0 - weighting
        self._old = weighting

        self._high_tick = None
        self._period = None

        pi.set_mode(gpio, pigpio.INPUT)

        self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._cbf)
        pi.set_watchdog(gpio, self._watchdog)

    def _cbf(self, gpio, level, tick):
        if level == 1:

            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)

                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)

                else:
                    self._period = t

            self._high_tick = tick

        elif level == 2:

            if self._period is not None:
                if self._period  < 2000000000:
                    self._period += (self._watchdog * 1000)
        
    def PWM(self, duty):
        self.pi.hardware_PWM(self.pwm, 25000, duty * 10000)
        
    def RPM(self):

        RPM = 0.0
        if self._period is not None:
            RPM = 60000000.0 / (self._period * self.pulses_per_rev)
            if RPM < self.min_RPM:
                RPM = 0.0
        return RPM

    def cancel(self):
        self.pi.hardware_PWM(self.pwm, 25000, 0)
        self.pi.set_watchdog(self.gpio, 0)
        self._cb.cancel() 


def start_sequence():
    print('\033c')
    print("*****************************")
    print(f"NURO FAN TESTING - {FILE_OUTPUT_NAME}")
    print("*****************************\n")

    MC_start = MotorController(PWM_PIN, MOTOR_EN_PIN, 0, 0)

    MC_start.bcm2835_init_spi()

    print("Waiting on motor board to power up...")
    print("(NOTE: Hold CTRL + 'C' to exit program)\n")

    try:
        while(MC_start.bcm2835_motor_ping()):
            pass
        #print('\033c')
        print("*****************************")
        print("Motor Board Connected!")
        print("*****************************")

        #end_sequence(MC_start)
        
        return 1

    except KeyboardInterrupt:
        end_sequence(MC_start)

        return 0

def run_main():

    import time
    import pigpio
    import fan_main

    RPM_GPIO = 4
    PWM_GPIO = 19
    RUN_TIME = int(input("Enter Duration: "))
    DUTY = int(input("Enter Duty Cycle %: "))
    SAMPLE_TIME = 0.5

    pi = pigpio.pi()

    p = fan_main.reader(pi, RPM_GPIO, PWM_GPIO)
    
    p.PWM(DUTY)

    start = time.time()

    while (time.time() - start) < RUN_TIME:
        try:
        
            time.sleep(SAMPLE_TIME)

            RPM = p.RPM()

            print("RPM = {}".format(int(RPM+0.5)))
        
        except KeyboardInterrupt:
            p.cancel()
            sys.exit()
        
        finally:
            pass

    p.cancel()

    #p.stop()

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