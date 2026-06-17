import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gpiozero import DigitalOutputDevice
import time
import math
import threading

class StepperHardwarePWM(Node):
    def __init__(self):
        super().__init__('stepper_hardware_pwm')

        self.dir_pin = DigitalOutputDevice(27)
        self.enable_pin = DigitalOutputDevice(22)
        self.enable_pin.off()

        self.target_steps = 0
        self.current_steps = 0
        self.lock = threading.Lock()
        self.running = True
        self.last_encoder_pos = None
        self.encoder_turns = 0
        self.unwrapped_pos = 0.0
        self.step_pin = DigitalOutputDevice(12)
        self._pwm_frequency = 0.0
        self._pwm_running = False
        self._pwm_lock = threading.Lock()
        self._pwm_thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._pwm_thread.start()
        self.STEPS_PER_REVOLUTION = 200
        self.MICROSTEPS = 8
        self.STEPS_PER_RADIANS = (self.STEPS_PER_REVOLUTION * self.MICROSTEPS) / (2 * math.pi)
        self.ENCODER_TO_STEPPER_SCALE = 1.0

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        self.motor_thread = threading.Thread(target=self._motor_control_loop, daemon=True)
        self.motor_thread.start()
        self.get_logger().info("Silnik uruchomiony w trybie sprzętowego PWM na GPIO 12.")
        
    def listener_callback(self, msg):
        if len(msg.position) == 0:
            return

        raw_pos = msg.position[0]
        TWO_PI = 2.0 * math.pi

        with self.lock:
            if self.last_encoder_pos is None:
                self.last_encoder_pos = raw_pos
                self.unwrapped_pos = raw_pos
            else:
                diff = raw_pos - self.last_encoder_pos
                if diff > math.pi:
                    self.encoder_turns -= 1
                elif diff < -math.pi:
                    self.encoder_turns += 1

                self.unwrapped_pos = raw_pos + self.encoder_turns * TWO_PI
                self.last_encoder_pos = raw_pos

            self.target_steps = int(
                self.unwrapped_pos * self.STEPS_PER_RADIANS * self.ENCODER_TO_STEPPER_SCALE
            )

    def _motor_control_loop(self):
        MAX_FREQ = 3000.0
        
        idle_counter = 0
        IDLE_THRESHOLD = 250

        while self.running:
            with self.lock:
                target = self.target_steps
                current = self.current_steps
            
            diff = target - current
            
            if diff != 0:
                idle_counter = 0
                if self.enable_pin.value:
                    self.enable_pin.off()

                if diff > 0:
                    self.dir_pin.on()
                    step_dir = 1
                else:
                    self.dir_pin.off()
                    step_dir = -1
                
                abs_diff = abs(diff)
                frequency = min(abs_diff * 6, MAX_FREQ)

                if frequency < 20:
                    frequency = 20

                # 3. Uruchomienie/Aktualizacja fali PWM (software)
                with self._pwm_lock:
                    self._pwm_frequency = frequency
                    self._pwm_running = True
                
                time.sleep(0.002)
                
                actual_steps_done = int(frequency * 0.002)
                if actual_steps_done == 0: 
                    actual_steps_done = 1
                    
                actual_steps_done = min(actual_steps_done, abs_diff)
                
                with self.lock:
                    self.current_steps += step_dir * actual_steps_done
            else:
                with self._pwm_lock:
                    self._pwm_running = False
                time.sleep(0.002)

                idle_counter += 1
                if idle_counter >= IDLE_THRESHOLD:
                    self.enable_pin.on()

    def _pwm_loop(self):
        last_toggle = time.time()
        state = False
        while self.running:
            with self._pwm_lock:
                running = self._pwm_running
                freq = self._pwm_frequency

            if not running or freq <= 0.0:
                time.sleep(0.001)
                last_toggle = time.time()
                state = False
                self.step_pin.off()
                continue

            period = 1.0 / freq
            half_period = period / 2.0

            now = time.time()
            if now - last_toggle >= half_period:
                state = not state
                if state:
                    self.step_pin.on()
                else:
                    self.step_pin.off()
                last_toggle = now

            time.sleep(0.0005)

    def destroy_node(self):
        self.running = False
        self.motor_thread.join()
        with self._pwm_lock:
            self._pwm_running = False
            self._pwm_frequency = 0.0
        self._pwm_thread.join()
        self.enable_pin.on()
        self.dir_pin.close()
        self.enable_pin.close()
        self.step_pin.close()
        super().destroy_node()

def main():
    rclpy.init()
    stepper = StepperHardwarePWM()
    try:
        rclpy.spin(stepper)
    except KeyboardInterrupt:
        pass
    finally:
        stepper.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
