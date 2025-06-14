import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import tkinter as tk

COLORS = {
    'obj_001': 'red',
    'obj_002': 'blue',
    'obj_003': 'orange',
    'obj_004': 'purple',
}

DROP_ZONES = {
    'obj_001': 100,
    'obj_002': 200,
    'obj_003': 300,
    'obj_004': 400,
}

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_gui')
        self.subscription = self.create_subscription(
            String, 'detected_object', self.object_detected_callback, 10)
        self.cli = self.create_client(AddTwoInts, 'pickup_object')

        self.obj_id = None
        self.object_attached = False

        # Interface
        self.root = tk.Tk()
        self.root.title("Superviseur - Transport d’objet")

        self.canvas = tk.Canvas(self.root, width=600, height=300, bg='white')
        self.canvas.pack()

        # Zones
        self.robot_start = 50
        self.object_pos = 400

        # Robot
        self.robot = self.canvas.create_oval(self.robot_start, 80, self.robot_start+30, 110, fill='blue', tags='robot')
        self.object = None

        # Zones de dépôt
        for obj, pos in DROP_ZONES.items():
            self.canvas.create_rectangle(pos, 250, pos+30, 270, fill=COLORS[obj])

        # État
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
        if self.obj_id is not None:
            return  # Ignore nouvelle détection si déjà en cours
        self.obj_id = msg.data
        self.log(f"[SUPERVISEUR] Objet détecté : {self.obj_id}")
        color = COLORS.get(self.obj_id, 'grey')
        self.object = self.canvas.create_rectangle(
            self.object_pos, 85, self.object_pos+20, 105, fill=color, tags='object')
        self.move_robot_to_object()

    def move_robot_to_object(self):
        x1, _, x2, _ = self.canvas.coords(self.robot)
        if x2 < self.object_pos:
            self.canvas.move(self.robot, 5, 0)
            self.root.after(50, self.move_robot_to_object)
        else:
            self.call_arm_service()

    def call_arm_service(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.log('Service pickup_object non disponible...')
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
            self.return_to_drop_zone()
        except Exception as e:
            self.log(f"[ERREUR] Service échoué : {e}")
            self.obj_id = None

    def return_to_drop_zone(self):
        x1, _, _, _ = self.canvas.coords(self.robot)
        target_x = DROP_ZONES.get(self.obj_id, 100)
        if x1 > target_x:
            self.canvas.move(self.robot, -5, 0)
            if self.object_attached:
                self.canvas.move(self.object, -5, 0)
            self.root.after(50, self.return_to_drop_zone)
        else:
            self.object_attached = False
            self.canvas.delete(self.object)
            self.log(f"[SUPERVISEUR] Objet {self.obj_id} déposé ✅")
            self.obj_id = None

def main(args=None):
    rclpy.init(args=args)
    supervisor = SupervisorNode()
    supervisor.root.mainloop()
    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
