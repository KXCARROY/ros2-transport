import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import tkinter as tk
import random
import math

COLORS = {
    'obj_001': 'red',
    'obj_002': 'green',
    'obj_003': 'orange',
    'obj_004': 'purple',
}

DROP_ZONES = {
    'obj_001': (100, 250),
    'obj_002': (200, 250),
    'obj_003': (300, 250),
    'obj_004': (400, 250),
}

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_gui')
        self.subscription = self.create_subscription(
            String, 'detected_object', self.object_detected_callback, 10)
        self.cli = self.create_client(AddTwoInts, 'pickup_object')

        self.obj_id = None
        self.object_attached = False
        self.object = None
        self.object_pos = (0, 0)
        self.target_pos = None

        # Interface
        self.root = tk.Tk()
        self.root.title("Superviseur XY tp introduction")

        self.canvas = tk.Canvas(self.root, width=600, height=300, bg='white')
        self.canvas.pack()

        # Robot
        self.robot = self.canvas.create_oval(50, 50, 80, 80, fill='blue')

        # Zones de dépôt
        for obj, (x, y) in DROP_ZONES.items():
            self.canvas.create_rectangle(x, y, x + 30, y + 20, fill=COLORS[obj])

        # Statut
        self.status_label = tk.Label(self.root, text="En attente d'objet...")
        self.status_label.pack()
        self.log_box = tk.Text(self.root, height=6, width=80)
        self.log_box.pack()

        self.root.after(100, self.spin_ros)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(50, self.spin_ros)

    def log(self, message):
        self.get_logger().info(message)
        self.log_box.insert(tk.END, message + '\n')
        self.log_box.see(tk.END)
        self.status_label.config(text=message)

    def object_detected_callback(self, msg):
        if self.obj_id is not None:
            return
        self.obj_id = msg.data
        self.log(f"[SUPERVISEUR] Objet détecté : {self.obj_id}")

        # Position aléatoire contrôlée
        x = random.randint(100, 500)
        y = random.randint(80, 200)
        self.object_pos = (x, y)
        color = COLORS.get(self.obj_id, 'grey')
        self.object = self.canvas.create_rectangle(x, y, x + 20, y + 20, fill=color)
        self.target_pos = (x, y)
        self.move_robot_to_target()

    def move_robot_to_target(self):
        rx, ry, rx2, ry2 = self.canvas.coords(self.robot)
        rcx = (rx + rx2) / 2
        rcy = (ry + ry2) / 2
        tx, ty = self.target_pos

        dx = tx - rcx
        dy = ty - rcy
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 2:
            self.call_arm_service()
            return

        step = 3
        dx_norm = (dx / distance) * step
        dy_norm = (dy / distance) * step

        self.canvas.move(self.robot, dx_norm, dy_norm)
        self.root.after(15, self.move_robot_to_target)

    def call_arm_service(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.log('Attente du service pickup_object...')
        request = AddTwoInts.Request()
        request.a = len(self.obj_id)
        request.b = 5
        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.after_pickup)

    def after_pickup(self, future):
        try:
            result = future.result()
            self.log(f"[SUPERVISEUR] Objet saisi (somme={result.sum})")
            self.object_attached = True
            self.target_pos = DROP_ZONES[self.obj_id]
            self.move_to_drop_zone()
        except Exception as e:
            self.log(f"[ERREUR] Saisie échouée : {e}")
            self.obj_id = None

    def move_to_drop_zone(self):
       rx1, ry1, rx2, ry2 = self.canvas.coords(self.robot)
       rcx = (rx1 + rx2) / 2
       rcy = (ry1 + ry2) / 2
       tx, ty = self.target_pos

       dx = tx - rcx
       dy = ty - rcy
       distance = math.sqrt(dx**2 + dy**2)

       if distance < 2:
           self.object_attached = False
           self.canvas.delete(self.object)
           self.log(f"[SUPERVISEUR] Objet {self.obj_id} déposé ✅")
           self.obj_id = None
           return

       step = 3
       dx_norm = (dx / distance) * step
       dy_norm = (dy / distance) * step

       # Déplacer le robot
       self.canvas.move(self.robot, dx_norm, dy_norm)

       # L’objet est "porté" sur le robot : on le repositionne exactement dessus
       if self.object_attached and self.object:
           rob_x1, rob_y1, rob_x2, rob_y2 = self.canvas.coords(self.robot)
           obj_x = (rob_x1 + rob_x2) / 2 - 10
           obj_y = (rob_y1 + rob_y2) / 2 - 10
           self.canvas.coords(self.object, obj_x, obj_y, obj_x + 20, obj_y + 20)

       self.root.after(15, self.move_to_drop_zone)

def main(args=None):
    rclpy.init(args=args)
    supervisor = SupervisorNode()
    supervisor.root.mainloop()
    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
