from optimal_control.utils import *
from optimal_control.environment import *

import matplotlib.pyplot as plt
import matplotlib.lines as lines

import matplotlib

class Plotter:

    def __init__(self, 
        po : PlotOptions,
        trajectories : list[list[State]] = None,
        environment : list[Region] = None,
        waypoints : list[list[State]] = None
    ):
        self.trajectories = trajectories
        self.waypoints = waypoints

        # Common jargon
        matplotlib.rcParams["figure.dpi"] = 300

        matplotlib.rcParams["font.family"] = "serif"
        matplotlib.rcParams["font.size"] = 8
        # matplotlib.rcParams["font.serif"] = "Times New Roman"
        self.figure, self.axis = plt.subplots(1, 1)
        self.axis.axis("equal")

        self.colors =  ["royalblue", "seagreen", "red", "black",  "brown", "purple"]

        if trajectories != None:
            self.paths = [
                # Returns the line, so this is acceptable
                self.axis.add_line(
                    lines.Line2D(
                        [], [],
                        linestyle="-",
                        color=self.colors[_],
                        linewidth=0.7
                    )
                ) for _ in range(len(self.trajectories))
            ]
            # self.start_points = [
            #     self.axis.plot(
            #         self.trajectories[_][0].x, self.trajectories[_][0].y, marker="o", markersize=4.0, color=self.colors[_]
            #     ) for _ in range(len(self.trajectories))
            # ]

        if self.waypoints != None:
            self.sparse_paths = [
                self.axis.add_line(
                    lines.Line2D(
                        [], [],
                        linestyle=" ",
                        marker="o",
                        markersize=2.0,
                        color = self.colors[_]
                    )
                ) for _ in range(len(self.waypoints))
            ]

        if environment != None:
            for region in environment:
                self.axis.add_patch(region.plot())

        self.axis.set_xlim(po.xlim); self.axis.set_ylim(po.ylim)
        self.axis.set_xlabel("X"); self.axis.set_ylabel("Y")
        # self.axis.set_title(po.title)

    def show_animation(self, record = False):
        if record:
            import io
            import imageio
            self.frames = []
        self.timer = self.figure.canvas.new_timer(interval=100)
        self.path_idx = 0; self.done = 0

        def callback():
            termination = (self.trajectories == None) or (self.done == len(self.trajectories) - 1)
            
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
                try:
                    x.append(self.trajectories[k][self.path_idx].x)
                    y.append(self.trajectories[k][self.path_idx].y)
                    self.paths[k].set_data(x, y)
                except:
                    self.done += 1

            self.figure.canvas.draw()
            self.path_idx += 1

            return True

        self.timer.add_callback(callback)
        self.timer.start()
        plt.show()

        if record:
            imageio.mimsave("example.mp4", self.frames[1:])

    def show_plot(self, plot=True):
        # Get the number of points in the trajectory
        if self.trajectories != None:
            for k in range(len(self.trajectories)):
                x = []; y = []
                for state in self.trajectories[k]:
                    x.append(state.x); y.append(state.y)
                self.paths[k].set_data(x, y)

        if self.waypoints != None:
            for k in range(len(self.waypoints)):
                x = []; y = []
                for state in self.waypoints[k]:
                    x.append(state.x); y.append(state.y)
                self.sparse_paths[k].set_data(x, y)
        
        if plot:
            plt.show()