#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

# ###########################################################################
# ## OSTATECZNE WARTOŚCI KALIBRACYJNE
# ###########################################################################

# Ograniczenie prędkości do niezawodnego poziomu, przy którym enkodery nie gubią impulsów.
MAKSYMALNA_PREDKOSC = 0.60

# Współczynnik do spowolnienia szybszej, prawej strony prawidłowo około 0.9 daje najlepsze wyyniki i robot jedzie w miare prosto.
WSPOLCZYNNIK_KOREKCYJNY_PRAWEJ_STRONY = 0.9
WSPOLCZYNNIK_KOREKCYJNY_OBROTOW = 0.7

# ###########################################################################
# ## KONFIGURACJA SPRZĘTOWA
# ###########################################################################

# Piny sterujące dla lewej strony (w kodzie jako Motor 1)
MOTOR1_PWM_PIN = 12
MOTOR1_DIR_A_PIN = 5
MOTOR1_DIR_B_PIN = 6

# Piny sterujące dla prawej strony (w kodzie jako Motor 2)
MOTOR2_PWM_PIN = 13
MOTOR2_DIR_A_PIN = 23
MOTOR2_DIR_B_PIN = 24

PWM_FREQUENCY = 100

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Inicjalizacja węzła sterownika silników...')

        # Inicjalizacja GPIO
        self.motor1_pwm = None
        self.motor2_pwm = None
        self.setup_gpio()

        # Subskrypcja tematu /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Mechanizm bezpieczeństwa (timeout)
        self.timeout_seconds = 1.0
        self.last_msg_time = self.get_clock().now()
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.is_stopped_by_timeout = False

        self.get_logger().info('Węzeł gotowy. Nasłuchuję na /cmd_vel...')

    def setup_gpio(self):
        """Konfiguruje piny GPIO dla silników."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([MOTOR1_PWM_PIN, MOTOR2_PWM_PIN, MOTOR1_DIR_A_PIN, MOTOR1_DIR_B_PIN, MOTOR2_DIR_A_PIN, MOTOR2_DIR_B_PIN], GPIO.OUT)
        self.motor1_pwm = GPIO.PWM(MOTOR1_PWM_PIN, PWM_FREQUENCY)
        self.motor2_pwm = GPIO.PWM(MOTOR2_PWM_PIN, PWM_FREQUENCY)
        self.motor1_pwm.start(0)
        self.motor2_pwm.start(0)

    def cmd_vel_callback(self, msg):
        """Callback wywoływany po otrzymaniu wiadomości na /cmd_vel."""
        self.last_msg_time = self.get_clock().now()
        self.is_stopped_by_timeout = False

        linear_x = msg.linear.x
        angular_z = msg.angular.z * WSPOLCZYNNIK_KOREKCYJNY_OBROTOW

        # 1. Obliczenie bazowych prędkości dla obu stron
        right_speed = linear_x + angular_z
        left_speed = linear_x - angular_z

        # 2. Zastosowanie współczynnika kalibracyjnego dla prawej strony
        right_speed *= WSPOLCZYNNIK_KOREKCYJNY_PRAWEJ_STRONY

        # 3. Zastosowanie globalnego ograniczenia prędkości
        right_speed *= MAKSYMALNA_PREDKOSC
        left_speed *= MAKSYMALNA_PREDKOSC

        # 4. Ostateczne zabezpieczenie przed przekroczeniem zakresu
        right_speed = max(min(right_speed, 1.0), -1.0)
        left_speed = max(min(left_speed, 1.0), -1.0)
        
        # 5. Ustawienie silników (z uwzględnieniem ewentualnej inwersji)
        self.set_motors(-left_speed, -right_speed)

    def check_timeout(self):
        """Sprawdza, czy od ostatniej wiadomości minął limit czasu."""
        elapsed_time = self.get_clock().now() - self.last_msg_time
        
        if elapsed_time > Duration(seconds=self.timeout_seconds):
            if not self.is_stopped_by_timeout:
                self.get_logger().warn(f'Timeout! Brak komendy od {self.timeout_seconds}s. Zatrzymywanie robota.')
                self.set_motors(0.0, 0.0)
                self.is_stopped_by_timeout = True

    def set_motors(self, left_speed, right_speed):
        """Ustawia prędkość i kierunek dla obu silników."""
        # --- Silnik lewy (Motor 1) ---
        if left_speed >= 0:
            GPIO.output(MOTOR1_DIR_A_PIN, GPIO.HIGH)
            GPIO.output(MOTOR1_DIR_B_PIN, GPIO.LOW)
        else:
            GPIO.output(MOTOR1_DIR_A_PIN, GPIO.LOW)
            GPIO.output(MOTOR1_DIR_B_PIN, GPIO.HIGH)
        self.motor1_pwm.ChangeDutyCycle(abs(left_speed) * 100)

        # --- Silnik prawy (Motor 2) ---
        if right_speed >= 0:
            GPIO.output(MOTOR2_DIR_A_PIN, GPIO.HIGH)
            GPIO.output(MOTOR2_DIR_B_PIN, GPIO.LOW)
        else:
            GPIO.output(MOTOR2_DIR_A_PIN, GPIO.LOW)
            GPIO.output(MOTOR2_DIR_B_PIN, GPIO.HIGH)
        self.motor2_pwm.ChangeDutyCycle(abs(right_speed) * 100)

    def destroy_node(self):
        """Funkcja wywoływana przy zamykaniu węzła."""
        self.get_logger().info("Zamykanie węzła i czyszczenie GPIO.")
        if self.motor1_pwm: self.motor1_pwm.stop()
        if self.motor2_pwm: self.motor2_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()