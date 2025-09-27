#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import threading

def euler_from_quaternion(x, y, z, w):
    """Konwertuje kwaternion na kąt Yaw w radianach."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_angle_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # zmienne do zliczania obrotów
        self.rotation_count = 0
        self.last_yaw_deg = 0.0
        self.current_yaw_deg = 0.0
        self.saved_results = []
        self.lock = threading.Lock()

        self.get_logger().info('Uruchomiono subskrybenta odometrii.')
        self.get_logger().info('Naciśnij [Enter], aby zapisać wynik.')
        self.get_logger().info('Naciśnij CTRL+C, aby zakończyć i zobaczyć podsumowanie.')

        # Wątek do nasłuchiwania klawisza [Enter]
        self.input_thread = threading.Thread(target=self.key_capture_thread, daemon=True)
        self.input_thread.start()

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        x, y, z, w = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        
        yaw_rad = euler_from_quaternion(x, y, z, w)
        yaw_deg = math.degrees(yaw_rad)
        
        
        with self.lock:
            if (self.last_yaw_deg > 150.0 and yaw_deg < -150.0):
                self.rotation_count += 1
            elif (self.last_yaw_deg < -150.0 and yaw_deg > 150.0):
                self.rotation_count -= 1
            
            # Zapisz aktualne wartości
            self.last_yaw_deg = yaw_deg
            self.current_yaw_deg = yaw_deg
        
        # Wyświetl wynik w czytelny sposób
        print(f"Aktualny kąt: {yaw_deg:<7.2f}° | Pełne obroty: {self.rotation_count:<3}   \r", end="")

    def key_capture_thread(self):
        """Wątek, który czeka na wciśnięcie [Enter] i zapisuje wynik."""
        while rclpy.ok():
            try:
                input() # Czekaj na Enter
                with self.lock:
                    # Oblicz i skopiuj aktualne wartości
                    total_angle_deg = self.rotation_count * 360 + self.current_yaw_deg
                    rotations = total_angle_deg / 360.0
                
                # Zapisz wynik
                self.saved_results.append({'stopnie': total_angle_deg, 'obroty': rotations})
                print(f"\n--- ZAPISANO WYNIK: {total_angle_deg:.2f}° ({rotations:.2f} obrotu) ---\n")

            except (EOFError, KeyboardInterrupt):
                break

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n--- PODSUMOWANIE ZAPISANYCH POMIARÓW ---")
        if odom_subscriber.saved_results:
            for i, result in enumerate(odom_subscriber.saved_results):
                print(f"Pomiar #{i+1}: Kąt = {result['stopnie']:.2f}°, Obroty = {result['obroty']:.2f}")
        else:
            print("Brak zapisanych pomiarów.")
        print("-" * 45)
        
        print("Zakończono działanie.")
        odom_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()