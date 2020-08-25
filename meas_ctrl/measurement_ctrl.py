import vna.vna_comms as vna_comms
import qpt.positioner as positioner
from qpt.integer import Coordinate
import data.data_storage as data_storage
from qpt.packet_parser import Parser
from time import sleep
from threading import Lock, Thread



class meas_ctrl:
    def __init__(
            self,
            freq,
            impedance=False,
            cal=False, 
            avg=8, 
            sweep_mode='step', 
            offset=0, 
            exe_mode='pan', 
            const_angle=0, 
            resolution=5, 
            file='data\\data0.csv'):
        self.impedance = impedance # if true, S11 and S21 will be measured. Else, only S21
        self.freq = freq # list or vna_comms.lin_freq obj
        self.cal = cal # true or false
        self.avg = avg # e.g. 8, 16, etc.
        self.sweep_mode = sweep_mode # either 'continuous' or 'step'
        self.offset = offset        
        self.exe_mode = exe_mode # 'pan' for pan sweep or 'tilt' for tilt sweep
        self.const_angle = const_angle # angle at which non-changing coordinate is set to
        self.resolution = resolution
        self.vna = vna_comms.session('GPIB0::16::INSTR')
        self.qpt = positioner.Positioner()
        self.progress = 0 # percentage, e.g. 0.11 for 11%
        self.vna_avg_delay = 0
        self.vna_S11_delay = 0
        self.vna_S21_delay = 0
        self.MAX_PAN_TIME = 1240
        self.MAX_TILT_TIME = 700
        self.MIN_PAN_SPEED = 8
        self.MIN_TILT_SPEED = 17
        self.MAX_PAN_SPEED = 127
        self.MAX_TILT_SPEED = 127
        self.pan_speed = 0
        self.tilt_speed = 0
        self.vna_lock = Lock()
        self.p = Parser()
        self.file = file

    def setup(self):
        if self.exe_mode == 'pan':
            self.qpt.move_to(-180+self.offset, self.const_angle, 'abs')
        else:
            self.qpt.move_to(self.offset+self.const_angle, -90, 'abs')
        
        self.vna.reset()
        data_storage.create_file(self.file)
        
        if self.cal == True:
            self.vna.calibrate() # cal prompts have to be changed for GUI integration
        
        self.vna.setup(self.freq, self.avg, 3700)
        [self.vna_avg_delay, self.vna_S11_delay, self.vna_S21_delay] = self.compute_vna_delay()
        
        if self.sweep_mode == 'continuous': # check if a continuous sweep is possible
            if self.exe_mode == 'pan':
                total_time = (self.vna_avg_delay + self.vna_S21_delay) * 360 / self.resolution
                if total_time > self.MAX_PAN_TIME:
                    self.sweep_mode = 'step'
                    self.pan_speed = 0
                else:
                    self.pan_speed = self.compute_pan_speed(total_time)
            else:
                total_time = (self.vna_avg_delay + self.vna_S21_delay) * 180 / self.resolution
                if total_time > self.MAX_TILT_TIME:
                    self.sweep_mode = 'step'
                    self.tilt_speed = 0
                else:
                    self.tilt_speed = self.compute_tilt_speed(total_time)


    def run(self):
        if self.impedance == True:
            self.vna.rst_avg('S11')
            sleep(self.vna_avg_delay)
            self.record_data('S11', self.file)    # need to create_file prior

        # Step Case
        if self.sweep_mode == 'step':
            # Pan Case
            if self.exe_mode == 'pan':
                for i in range(0, int(360/self.resolution) + 1):
                    self.step_delay()
                    self.record_data('S21', self.file)
                    self.progress = (i+1) * self.resolution / 360
                    if self.is_step_pan_complete() is True:
                        break
            # Tilt Case
            else:
                for i in range(0, int(180/self.resolution) + 1):
                    self.step_delay()
                    self.record_data('S21', self.file)
                    self.progress = (i+1) * self.resolution / 180
                    if self.is_step_tilt_complete() is True:
                        break

        # Continuous Case
        else:
            # Pan Case
            if self.exe_mode == 'pan':
                self.init_continuous_sweep()
                for i in range(0, int(360/self.resolution) + 1):
                    lock = self.init_continuous_lock()
                    target = ((i+1) * self.resolution) - 180
                    curr = self.qpt.get_position()
                    while lock.acquire(blocking=False) is not True:
                        while curr.pan_angle() < target:
                            sleep(.08)
                            self.qpt.jog_cw(self.pan_speed, Coordinate(180,0))
                            curr = self.qpt.get_position()
                    self.record_data('S21', self.file)
                    self.progress = (target + 180) / 360
                    if self.is_continuous_pan_complete() is True:
                        break
            # Tilt Case
            else:
                self.init_continuous_sweep()
                for i in range(0, int(180/self.resolution) + 1):
                    lock = self.init_continuous_lock()
                    target = ((i+1) * self.resolution) - 90
                    curr = self.qpt.get_position()
                    while lock.acquire(blocking=False) is not True:
                        while curr.tilt_angle() < target:
                            sleep(.08)
                            self.qpt.jog_up(self.tilt_speed, Coordinate(0,90))
                            curr = self.qpt.get_position()
                    self.record_data('S21', self.file)
                    self.progress = (target + 90) / 180
                    if self.is_continuous_tilt_complete() is True:
                        break                    


    def halt(self):
        self.qpt.move_to(0, 0, 'stop')


    def record_data(self, S, file):
        if S == 'S21':
            curr = self.qpt.get_position()
            data_storage.append_data(file, self.vna.get_data(curr.tilt_angle(), curr.pan_angle(), S))
        else:
            data_storage.append_data(file, self.vna.get_data(0, 0, S))


    def step_delay(self):
        self.vna.rst_avg('S21')
        sleep(self.vna_avg_delay)


    def continuous_delay(self, lock):
        with lock:
            sleep(self.vna_avg_delay)


    def is_step_pan_complete(self):
        if self.progress > 1:
            return True
        self.qpt.move_to(self.resolution, 0, 'delta')
        return False


    def is_step_tilt_complete(self):
        if self.progress > 1:
            return True
        self.qpt.move_to(0, self.resolution, 'delta')
        return False


    def init_continuous_sweep(self):
        lock = Lock()
        self.vna.rst_avg('S21')
        t = Thread(target=self.continuous_delay, args=(lock,))
        t.start()                
        t.join()
        self.record_data('S21', self.file)


    def init_continuous_lock(self):
        lock = Lock()
        self.vna.rst_avg('S21')
        t = Thread(target=self.continuous_delay, args=(lock,))
        t.start()
        return lock


    def is_continuous_pan_complete(self):
        if self.progress >= 1:
            self.halt()
            return True
        self.qpt.jog_cw(self.pan_speed, Coordinate(180,0))
        return False


    def is_continuous_tilt_complete(self):
        if self.progress >= 1:
            self.halt()
            return True
        self.qpt.jog_up(self.tilt_speed, Coordinate(0,90))
        return False


    # returns list w/ 3 numbers in seconds, [averaging delay, get_data delay (S11), get_data delay (S21)]
    def compute_vna_delay(self):        
        if isinstance(self.freq, list):
            if len(self.freq) <= 5:
                if self.avg <= 8:
                    return [2.19, 2.13, 1.28]
                return [3.98, 2.13, 1.28]
            elif len(self.freq) <= 10:
                if self.avg <= 8:
                    return [3.02, 2.34, 1.36]
                return [5.80, 2.34, 1.36]
            elif len(self.freq) <= 15:
                if self.avg <= 8:
                    return [3.11, 2.46, 1.46]
                return [5.95, 2.46, 1.46]
            elif len(self.freq) <= 20:
                if self.avg <= 8:
                    return [3.36, 2.60, 1.61]
                return [6.48, 2.60, 1.61]
            elif len(self.freq) <= 25:
                if self.avg <= 8:
                    return [3.26, 2.75, 1.60]
                return [6.35, 2.75, 1.60]
            else:
                if self.avg <= 8:
                    return [3.58, 2.79, 1.69]
                return [6.76, 2.79, 1.69]
        else:
            if self.freq.points <= 201:
                if self.avg <= 8:
                    return [3.79, 3.60, 2.54]
                return [7.30, 3.60, 2.54]
            elif self.freq.points <= 401:
                if self.avg <= 8:
                    return [4.23, 5.04, 3.76]
                return [7.99, 5.04, 3.76]
            elif self.freq.points <= 801:
                if self.avg <= 8:
                    return [5.49, 7.86, 6.24]
                return [10.39, 7.86, 6.24]
            elif self.freq.points <= 1601:
                if self.avg <= 8:
                    return [8.51, 13.57, 11.02]
                return [16.06, 13.57, 11.02]


    def compute_pan_speed(self, total_time):
        pan_speed = ((12.8866*(360.0 / total_time) + 3.1546) // 1)
        if pan_speed <= self.MIN_PAN_SPEED:
            return self.MIN_PAN_SPEED
        elif pan_speed >= self.MAX_PAN_SPEED:
            return self.MAX_PAN_SPEED
        return int(pan_speed)


    def compute_tilt_speed(self, total_time):
        tilt_speed = ((39.3701*(180.0 / total_time) + 6.8228) // 1)
        if tilt_speed <= self.MIN_TILT_SPEED:
            return self.MIN_TILT_SPEED
        elif tilt_speed >= self.MAX_TILT_SPEED:
            return self.MAX_TILT_SPEED
        return int(tilt_speed)
"""End meas_ctrl Class"""

