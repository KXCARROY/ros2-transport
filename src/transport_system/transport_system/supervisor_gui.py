#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import tkinter as tk

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_gui')
        self.subscription = self.create_subscription(
            String, 'detected_object', self.object_detected_callback, 10)
        self.cli = self.create_client(AddTwoInts, 'pickup_object')

        # Interface
        self.root = tk.Tk()
        self.root.title("Superviseur - Transport d’objet")

        self.canvas = tk.Canvas(self.root, width=600, height=200, bg='white')
        self.canvas.pack()

        # Points de référence
        self.robot_start = 50
        self.object_pos = 400
        self.drop_pos = 100

        # Éléments
        self.robot = self.canvas.create_oval(self.robot_start, 80, self.robot_start+30, 110, fill='blue', tags='robot')
        self.object = self.canvas.create_rectangle(self.object_pos, 85, self.object_pos+20, 105, fill='red', tags='object')
        self.drop_zone = self.canvas.create_rectangle(self.drop_pos, 150, self.drop_pos+40, 170, fill='lightgreen')
        self.status_label = tk.Label(self.root, text="En attente d'objet...")
        self.status_label.pack()

        self.log_box = tk.Text(self.root, height=6, width=80)
        self.log_box.pack()

        self.root.after(100, self.spin_ros)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_ros)

    def log(self, message):
        self.get_logger().info(message)
        self.log_box.insert(tk.END, message + '\n')
        self.log_box.see(tk.END)
        self.status_label.config(text=message)

    def object_detected_callback(self, msg):
        obj_id = msg.data
        self.log(f"[SUPERVISEUR] Objet détecté : {obj_id}")
        self.canvas.itemconfig(self.object, fill='red')
        self.call_arm_service(obj_id)

    def call_arm_service(self, obj_id):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.log('Service pickup_object non disponible, attente...')
        request = AddTwoInts.Request()
        request.a = len(obj_id)  # simulation de l’ID objet
        request.b = 5  # force
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.after_pickup)

    def after_pickup(self, future):
        try:
            result = future.result()
            self.log(f"[SUPERVISEUR] Objet saisi (somme={result.sum})")
            self.move_robot_to_object()
        except Exception as e:
            self.log(f"[ERREUR] Service échoué : {e}")

    def move_robot_to_object(self):
        x1, y1, x2, y2 = self.canvas.coords(self.robot)
        if x2 < self.object_pos:
            self.canvas.move(self.robot, 5, 0)
            self.root.after(50, self.move_robot_to_object)
        else:
            self.canvas.itemconfig(self.object, state='hidden')
            self.root.after(500, self.return_to_start)

    def return_to_start(self):
        x1, y1, x2, y2 = self.canvas.coords(self.robot)
        if x1 > self.drop_pos:
            self.canvas.move(self.robot, -5, 0)
            self.root.after(50, self.return_to_start)
        else:
            self.canvas.create_rectangle(self.drop_pos+10, 155, self.drop_pos+30, 165, fill='green', tags='depose')
            self.log("[SUPERVISEUR] Objet déposé en zone verte ✅")

def main(args=None):
    rclpy.init(args=args)
    supervisor = SupervisorNode()
    supervisor.root.mainloop()
    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

