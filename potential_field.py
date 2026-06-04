"""
potential_field.py
------------------
Energy-based force fields for haptic rendering.

The field is a scalar potential  U(x, y).
Forces are derived as  F = -∇U,  guaranteeing the field never injects energy.

Interior of the circle  →  U = 0      (flat valley, zero force)
Outside the wall        →  U = k · p  where p is radial penetration depth
                            F = k · (-r̂)  constant magnitude, directed radially inward

Usage in Main.py
----------------
    from potential_field import CircleField
    field = CircleField(center=[0.10, 0.35], radius=0.025, k=200.0)
    ...
    force = field(pos[0], pos[1])              # callable — returns np.array([Fx, Fy])
    force = field(pos[0], pos[1], plots=True)  # same, and saves field_energy/forces.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')          # no GUI / no tkinter needed
import matplotlib.pyplot as plt


class CircleField:
    """
    Circular potential-energy field centered at `center` with `radius`.

    Parameters
    ----------
    center : array-like (2,)   center of the circle in metres
    radius : float             radius of the safe zone in metres
    k      : float             wall stiffness [N/m]
    """

    def __init__(self, center, radius, k=200.0):
        self.center = np.asarray(center, dtype=float)
        self.r = float(radius)
        self.k = float(k)

    # ------------------------------------------------------------------
    # Core field functions (accept scalars or numpy arrays)
    # ------------------------------------------------------------------

    def energy(self, x, y):
        """
        Scalar potential energy U(x, y) [J].
        Accepts scalars or same-shape numpy arrays (useful for plotting).
        """
        x, y = np.asarray(x, float), np.asarray(y, float)
        cx, cy = self.center
        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        p = np.maximum(0.0, dist - self.r)
        return self.k * p   # U = k·p  (linear → constant-force wall)

    def force(self, x, y):
        """
        2D force vector F = -∇U at (x, y) [N].
        Returns np.array([Fx, Fy]).
        Inside the circle F = [0, 0]; outside, constant magnitude k radially inward.
        """
        cx, cy = self.center
        dx, dy = x - cx, y - cy
        dist = np.sqrt(dx**2 + dy**2)
        if dist <= self.r or dist == 0.0:
            return np.array([0.0, 0.0])
        r_hat = np.array([dx, dy]) / dist
        return -self.k * r_hat   # constant magnitude, radially inward

    def __call__(self, x, y, plots=False):
        """
        Evaluate the force at (x, y) and optionally save field plots.

        Parameters
        ----------
        x, y  : float   end-effector position [m]
        plots : bool    if True, save field_energy.png and field_forces.png

        Returns
        -------
        np.array([Fx, Fy])  [N]
        """
        f = self.force(x, y)
        if plots:
            fig_e, _ = self.plot_energy()
            fig_e.tight_layout()
            fig_e.savefig('field_energy.png', dpi=150)
            plt.close(fig_e)
            print('Saved → field_energy.png')

            fig_f, _ = self.plot_forces()
            fig_f.tight_layout()
            fig_f.savefig('field_forces.png', dpi=150)
            plt.close(fig_f)
            print('Saved → field_forces.png')
        return f

    def update(self, x, y):
        """Drop-in replacement for PDController.update(x, y)."""
        return self.force(x, y)

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def plot_energy(self, ax=None, xlim=None, ylim=None, n=100):
        """
        Colour-map of the scalar energy field U(x, y).

        Parameters
        ----------
        ax         : existing matplotlib Axes, or None to create a new figure
        xlim, ylim : (min, max) tuples; default = 3x radius around center
        n          : grid resolution

        Returns
        -------
        fig, ax - energy colour-map figure
        """
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
        """
        2-D quiver plot of F = -∇U on an nxn grid.

        Parameters
        ----------
        ax         : existing Axes, or None to create a new figure
        xlim, ylim : (min, max) tuples; default = 3x radius around center
        n          : quiver grid resolution (arrows per axis)

        Returns
        -------
        fig, ax - force quiver figure
        """
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
        """Overlay the circle boundary and center marker on an existing Axes."""
        cx, cy = self.center
        circle = plt.Circle((cx, cy), self.r,
                             edgecolor='lime', facecolor='none',
                             linewidth=2, linestyle='--', label='wall')
        ax.add_patch(circle)
        ax.plot(cx, cy, 'g+', markersize=10, label='center')


# ======================================================================
# Demo / self-test
# ======================================================================
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
