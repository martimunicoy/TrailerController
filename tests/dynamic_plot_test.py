import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib import animation

class InteractiveCircle(object):
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.patches = []
        self.patches.append(Rectangle(xy=(1, 1), width=1, height=2,
                            facecolor='blue', edgecolor='k',
                            linewidth=2))
        for patch in self.patches:
            self.ax.add_artist(patch)

    def update(self, i, positions):
        self.patches[0].set_xy(positions[i])
        #self.patches[0].set_transform(Affine2D().rotate_around(1,2,1) + self.ax.transData)
        return self.patches[0]

    def show(self):
        plt.show()


circle = InteractiveCircle()

positions = [[[1,1], [1,2], [1,3], [2,3], [3,3]]]
anim = animation.FuncAnimation(circle.fig, circle.update, frames=5, interval=1000, repeat=True, fargs=(positions))
#anim.save('test.mp4', fps=30, extra_args=['vcodec', 'libx264'])
circle.show()
