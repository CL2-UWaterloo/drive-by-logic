from optimal_control.utils import *
from optimal_control.environment import *

import matplotlib.pyplot as plt
import matplotlib.lines as lines

class Plotter:

    def __init__(self, po : PlotOptions, trajectories : list[list[State]] = None, environment : list[Region] = None):
        self.trajectories = trajectories

        # Common jargon
        self.figure, self.axis = plt.subplots(1, 1)

        self.axis.set_prop_cycle(color=['red', 'green', 'blue'])

        if trajectories != None:
            self.paths = [
                # Returns the line, so this is acceptable
                self.axis.add_line(
                    lines.Line2D(
                        [], [],
                        linestyle=" ",
                        marker="o",
                        markersize=2.0
                    )
                ) for _ in range(len(self.trajectories))
            ]

        if environment != None:
            for region in environment:
                self.axis.add_patch(region.plot())

        self.axis.set_xlim(po.xlim); self.axis.set_ylim(po.ylim)
        self.axis.set_xlabel("X"); self.axis.set_ylabel("Y")
        self.axis.set_title(po.title)

    def show_animation(self, record = False):
        if record:
            import io
            import imageio
            self.frames = []
        self.timer = self.figure.canvas.new_timer(interval=1)
        self.path_idx = 0

        def callback():
            termination = (self.trajectories == None) or (self.path_idx == len(self.trajectories[0]))
            
            if termination:
                self.figure.canvas.draw()
                return True
            
            if record:
                frame_buffer = io.BytesIO()
                self.figure.savefig(frame_buffer, format="raw"); frame_buffer.seek(0)
                self.frames.append(
                    np.reshape(
                        np.frombuffer(frame_buffer.getvalue(), dtype=np.uint8),
                        newshape=(int(self.figure.bbox.bounds[3]), int(self.figure.bbox.bounds[2]), -1)
                    )
                )
                frame_buffer.close()

            for k in range(len(self.trajectories)):
                x, y = self.paths[k].get_data()
                x.append(self.trajectories[k][self.path_idx].x)
                y.append(self.trajectories[k][self.path_idx].y)
                self.paths[k].set_data(x, y)
        
            self.figure.canvas.draw()
            self.path_idx += 1

            return True

        self.timer.add_callback(callback)
        self.timer.start()
        plt.show()

        if record:
            imageio.mimsave("example.mp4", self.frames)

    def show_plot(self):
        # Get the number of points in the trajectory
        if self.trajectories != None:
            for k in range(len(self.trajectories)):
                x = []; y = []
                for state in self.trajectories[k]:
                    x.append(state.x); y.append(state.y)
                self.paths[k].set_data(x, y)
        plt.show()