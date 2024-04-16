import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',  # Assuming the topic is named 'odometry'
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        self.x_data = []
        self.y_data = []
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label='Position')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()

    def callback(self, msg):
        position = msg.pose.pose.position
        self.x_data.append(position.x)
        self.y_data.append(position.y)

    def update_plot(self, frame):
        self.line.set_data(self.x_data, self.y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PlotterNode()

    ani = FuncAnimation(plotter_node.fig, plotter_node.update_plot, interval=100)

    try:
        rclpy.spin(plotter_node)
    except KeyboardInterrupt:
        pass

    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
