
import numpy as np
import matplotlib
matplotlib.use('Agg')  
import matplotlib.pyplot as plt


class CircleField:

    def __init__(self, center, radius, k=200.0):
        self.center = np.asarray(center, dtype=float)
        self.r = float(radius)
        self.k = float(k)

    def energy(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        cx, cy = self.center
        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        p = np.maximum(0.0, dist - self.r)
        return 0.5 * self.k * p**2   # U = ½k·p²  

    def force(self, x, y):
    
        cx, cy = self.center
        dx, dy = x - cx, y - cy
        dist = np.sqrt(dx**2 + dy**2)
        if dist <= self.r or dist == 0.0:
            return np.array([0.0, 0.0])
        r_hat = np.array([dx, dy]) / dist
        return -self.k * (dist - self.r) * r_hat   # linear stiffness: F = -k·p radially inward

    def __call__(self, x, y, plots=False):
     
        f = self.force(x, y)
        if plots:
            fig_e, _ = self.plot_energy()
            fig_e.tight_layout()
            fig_e.savefig('field_energy.png', dpi=150)
            plt.close(fig_e)
            print('Saved field_energy.png')

            fig_f, _ = self.plot_forces()
            fig_f.tight_layout()
            fig_f.savefig('field_forces.png', dpi=150)
            plt.close(fig_f)
            print('Saved field_forces.png')
        return f

    def update(self, x, y):
        return self.force(x, y)

    # Visualisation
   

    def plot_energy(self, ax=None, xlim=None, ylim=None, n=100):
        
        cx, cy = self.center
        r = self.r
        pad = 3.0 * r
        xlim = xlim or (cx - pad, cx + pad)
        ylim = ylim or (cy - pad, cy + pad)

        xs = np.linspace(*xlim, n)
        ys = np.linspace(*ylim, n)
        X, Y = np.meshgrid(xs, ys)
        U = self.energy(X, Y)

        if ax is None:
            fig, ax = plt.subplots(figsize=(6, 5))
        else:
            fig = ax.get_figure()

        pcm = ax.pcolormesh(X, Y, U, shading='auto', cmap='hot_r')
        fig.colorbar(pcm, ax=ax, label='Energy U  [J]')

        self._draw_circle(ax)
        ax.set_aspect('equal')
        ax.set_xlabel('x  [m]')
        ax.set_ylabel('y  [m]')
        ax.set_title(
            f'Energy field   center={tuple(self.center.round(4))}  '
            f'radius={r} m   k={self.k} N/m'
        )
        ax.legend(loc='upper right', fontsize=8)
        return fig, ax

    # ------------------------------------------------------------------
    def plot_forces(self, ax=None, xlim=None, ylim=None, n=24):
    
        cx, cy = self.center
        r = self.r
        pad = 3.0 * r
        xlim = xlim or (cx - pad, cx + pad)
        ylim = ylim or (cy - pad, cy + pad)

        xq = np.linspace(*xlim, n)
        yq = np.linspace(*ylim, n)
        Xq, Yq = np.meshgrid(xq, yq)

        # Vectorised force evaluation over the grid
        F = np.array([[self.force(x, y) for x in xq] for y in yq])
        Fxq, Fyq = F[..., 0], F[..., 1]
        mag = np.hypot(Fxq, Fyq)

        if ax is None:
            fig, ax = plt.subplots(figsize=(6, 5))
        else:
            fig = ax.get_figure()

        # Colour arrows by magnitude so zero-force interior is visually distinct
        q = ax.quiver(Xq, Yq, Fxq, Fyq, mag,
                      cmap='viridis', scale_units='xy',
                      scale=self.k * 100, width=0.004)
        fig.colorbar(q, ax=ax, label='|F|  [N]')

        self._draw_circle(ax)
        ax.set_aspect('equal')
        ax.set_xlabel('x  [m]')
        ax.set_ylabel('y  [m]')
        ax.set_title(
            f'Force field  F = −∇U   center={tuple(self.center.round(4))}  '
            f'radius={r} m   k={self.k} N/m'
        )
        ax.legend(loc='upper right', fontsize=8)
        return fig, ax

    # ------------------------------------------------------------------
    def _draw_circle(self, ax):
        cx, cy = self.center
        circle = plt.Circle((cx, cy), self.r,
                             edgecolor='lime', facecolor='none',
                             linewidth=2, linestyle='--', label='wall')
        ax.add_patch(circle)
        ax.plot(cx, cy, 'g+', markersize=10, label='center')


if __name__ == '__main__':
    field = CircleField(center=[0.10, 0.35], radius=0.025, k=200.0)

    print('--- Force spot-checks (callable interface) ---')
    print(f'  center        (0.10, 0.35) → {field(0.10, 0.35)}   (expect [0, 0])')
    print(f'  outside right (0.13, 0.35) → {field(0.13, 0.35)}   (expect Fx < 0)')
    print(f'  outside left  (0.07, 0.35) → {field(0.07, 0.35)}   (expect Fx > 0)')
    print(f'  outside top   (0.10, 0.38) → {field(0.10, 0.38)}   (expect Fy < 0)')
    print(f'  outside bot   (0.10, 0.32) → {field(0.10, 0.32)}   (expect Fy > 0)')
    print(f'  diagonal      (0.13, 0.38) → {field(0.13, 0.38)}   (expect Fx<0, Fy<0)')

    # plots=True saves field_energy.png and field_forces.png
    field(0.10, 0.35, plots=True)
