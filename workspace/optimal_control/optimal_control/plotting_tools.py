from optimal_control.utils import *

import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches
from matplotlib.legend_handler import HandlerPatch

def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
    return patches.FancyArrow(0, 0.5*height, width, 0, length_includes_head=True, head_width=0.75*height)

class ProcessPlotter:

    def __init__(self,
        init : list[State],
        planner,
        waypoint_logs : list
    ):
        self.state : list[State] = init

        self.waypoint_logs = waypoint_logs

        self.number_of_agents = planner.number_of_agents
        self.number_of_waypoints = planner.number_of_waypoints
        self.granularity = planner.granularity

        self.progress = 0
        self.waypoint_idx = 1
        self.granularity_idx = 0

        self.fig, self.ax = plt.subplots()

        self.ax.set_xlim(-5, 30)
        self.ax.set_ylim(-5, 30)

        self.legend_elements = []

        self.state_lines = []
        colors = ["#F28522", "#009ADE", "#FFAA00"]
        for agent in range(self.number_of_agents):
            self.state_lines.append(
                lines.Line2D(
                    [], [],
                    linestyle="--", color=colors[agent],
                    label="Robot " + str(agent) + " Path"
                )
            )

            self.legend_elements.append(
                lines.Line2D([], [], color=colors[agent], linestyle="--", label="Robot " + str(agent) + " Path"),
            )

            self.ax.add_line(self.state_lines[-1])

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        self.legend_elements.extend([
            lines.Line2D([], [], marker='o', color='red', label='Obstacles', linestyle=" ", markerfacecolor="w", markersize=10),
            lines.Line2D([], [], marker='o', color='black', label='Goals', linestyle=" ", markerfacecolor="w", markersize=10),
        ])

        for obstacle in planner.obstacles:
            self.ax.add_patch(patches.Circle(
                (obstacle[0], obstacle[1]), obstacle[2],
                linewidth=1, edgecolor='red', facecolor='none'
            ))
        
        for goal in planner.final:
            self.ax.add_patch(patches.Circle(
                (goal[0], goal[1]), 2,
                linewidth=1, edgecolor='black', facecolor='none'
            ))        

        self.frames = []

    def terminate(self):
        plt.close("all")
    
    def callback(self):
        # frame_buffer = io.BytesIO()
        # self.fig.savefig(frame_buffer, format="raw"); frame_buffer.seek(0)
        # self.frames.append(
        #     np.reshape(
        #         np.frombuffer(frame_buffer.getvalue(), dtype=np.uint8),
        #         newshape=(int(self.fig.bbox.bounds[3]), int(self.fig.bbox.bounds[2]), -1)
        #     )
        # )
        # frame_buffer.close()

        if self.progress == self.number_of_agents*(self.number_of_waypoints-1)*self.granularity:
            return True

        for adx in range(self.number_of_agents):
            v = float(self.waypoint_logs[adx*self.number_of_waypoints + self.waypoint_idx].v)
            k = float(self.waypoint_logs[adx*self.number_of_waypoints + self.waypoint_idx].k)
            dt = float(self.waypoint_logs[adx*self.number_of_waypoints + self.waypoint_idx].t)/self.granularity

            self.state[adx].theta += k*v*dt
            self.state[adx].x += v*cos(self.state[adx].theta)*dt
            self.state[adx].y += v*sin(self.state[adx].theta)*dt

            state_x, state_y = self.state_lines[adx].get_data()
            state_x.append(self.state[adx].x)
            state_y.append(self.state[adx].y)
            self.state_lines[adx].set_data(state_x, state_y)
            self.fig.canvas.draw()

            self.progress += 1

        self.granularity_idx += 1
        if self.granularity_idx == self.granularity:
            self.waypoint_idx += 1
            self.granularity_idx = 0

        return True

    def __call__(self):
        timer = self.fig.canvas.new_timer(interval=1)
        timer.add_callback(self.callback)
        timer.start()

        self.ax.legend(
            handles=self.legend_elements,
            handler_map={patches.FancyArrow : HandlerPatch(patch_func=make_legend_arrow)}
        )

        plt.show()

        # imageio.mimsave("example.mp4", self.frames)