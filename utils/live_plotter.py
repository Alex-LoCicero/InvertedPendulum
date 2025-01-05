import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LivePlotter:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.data = {}
        self.lines = {}
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

    def update(self, new_data):
        # Initialize data and lines if they are empty
        if not self.data:
            for key in new_data:
                self.data[key] = []
                if key != 'timestamp':
                    self.lines[key] = self.ax.plot([], [], label=key)[0]
            self.ax.legend()

        # Update data
        for key in new_data:
            if key in self.data:
                self.data[key].append(new_data[key])
            else:
                self.data[key] = [new_data[key]]
                if key != 'timestamp':
                    self.lines[key] = self.ax.plot([], [], label=key)[0]
                    self.ax.legend()

    def update_plot(self, frame):
        for key, line in self.lines.items():
            line.set_data(self.data['timestamp'], self.data[key])
        self.ax.relim()
        self.ax.autoscale_view()
        return self.lines.values()

    def show(self):
        plt.show()