"""
Ground geometry of the three laser emitters (pure functions, no ROS).

The vehicle carries three laser emitters, each tilted by the same half-angle
``theta`` away from straight down and spread 120 degrees apart around the
body z axis.  This module answers: *where do those beams hit the ground?*

Frames
------
Everything here is in PX4's local **NED** frame (x = North, y = East,
z = Down) and PX4's **FRD** body frame (x = forward, y = right, z = down).
:func:`ned_to_enu` converts a point for RViz, which expects ENU
(x = East, y = North, z = Up).

Derivation
----------
Let the vehicle body origin be at ``p = (pN, pE, pD)`` with heading ``psi``
(yaw, NED, 0 = North, positive clockwise seen from above) and let ``h`` be its
height above the ground directly below it (``vehicle_local_position.dist_bottom``).

Emitter ``k`` is mounted in the body frame at::

    m_k = ( r_e * cos(phi_k),  r_e * sin(phi_k),  z_e )        (FRD)

where ``r_e`` is the horizontal mount radius, ``z_e`` the mount offset below the
body origin (positive = lower) and ``phi_k`` its azimuth measured from the
body +x axis towards +y (right).  Its beam direction in the body frame is::

    d_k = ( sin(theta) * cos(phi_k),  sin(theta) * sin(phi_k),  cos(theta) )

i.e. a unit vector ``theta`` away from the body down axis, leaning outwards
along the same azimuth as the mount.

Assumption: the vehicle is hovering level (roll = pitch = 0), so the only
body-to-NED rotation is the yaw ``psi``::

    Rz(psi) * (bx, by) = ( bx*cos(psi) - by*sin(psi),  bx*sin(psi) + by*cos(psi) )

and the vertical components are unchanged.  The emitter therefore sits at

    E_k = p + Rz(psi) * m_k

and is ``h - z_e`` above the ground.  Walking along the beam until its
vertical component covers that height takes ``t = (h - z_e) / cos(theta)``
metres of beam, which moves ``(h - z_e) * tan(theta)`` horizontally.  The
ground contact point is::

    G_k = ( pN + R_k * cos(phi_k + psi),  pE + R_k * sin(phi_k + psi),  pD + h )
    R_k = r_e + (h - z_e) * tan(theta)

Because all three emitters share ``theta``, ``r_e`` and ``z_e``, every ``R_k``
is the same radius ``R`` and the contact points lie on a circle of radius
``R`` around the ground point below the vehicle, 120 degrees apart: an
equilateral triangle with side ``sqrt(3) * R``.

Why the *range-finder* height and not baro / GPS altitude
---------------------------------------------------------
``R`` depends on the distance from the emitters to the *surface the beams
actually land on*.  Barometric altitude is relative to the pressure at
arming and drifts with weather and rotor wash; GPS altitude is relative to
an ellipsoid with metres of error; neither knows where the ground is.  The
downward range finder measures exactly the quantity in the formula, so the
node uses ``dist_bottom`` (the EKF's range-aided height above ground) and
falls back to the raw range reading, never to ``-z``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

Vec3 = Tuple[float, float, float]

# Emitter k sits at azimuth k * 120 deg in the model SDF, which is expressed
# in Gazebo's x-forward / y-LEFT frame.  PX4's body frame has y to the RIGHT,
# so the same physical emitters are at 0, -120 and +120 degrees here.
DEFAULT_AZIMUTHS_FRD: Tuple[float, float, float] = (0.0, -2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0)


@dataclass(frozen=True)
class EmitterLayout:
    """How the emitters are mounted; mirrors laser_emitter_*_link in the SDF."""

    half_angle: float           # theta, radians, tilt of every beam from nadir
    mount_radius: float = 0.08  # r_e, metres from the body origin (horizontal)
    mount_down: float = 0.03    # z_e, metres below the body origin (FRD z)
    azimuths: Tuple[float, ...] = DEFAULT_AZIMUTHS_FRD

    def __post_init__(self) -> None:
        if not 0.0 <= self.half_angle < math.pi / 2.0:
            raise ValueError('half_angle must be in [0, 90) degrees')
        if self.mount_radius < 0.0:
            raise ValueError('mount_radius must be >= 0')


@dataclass(frozen=True)
class Beam:
    """One emitter and the point where its beam meets the ground, both NED."""

    emitter_ned: Vec3
    ground_ned: Vec3


def _rotate_yaw(bx: float, by: float, psi: float) -> Tuple[float, float]:
    """Rotate a body-frame horizontal vector (FRD) into NED by heading psi."""
    c, s = math.cos(psi), math.sin(psi)
    return bx * c - by * s, bx * s + by * c


def contact_radius(agl: float, layout: EmitterLayout) -> float:
    """R = r_e + (h - z_e) * tan(theta): horizontal reach of every beam."""
    return layout.mount_radius + (agl - layout.mount_down) * math.tan(layout.half_angle)


def triangle_side(agl: float, layout: EmitterLayout) -> float:
    """Side length of the equilateral ground triangle, sqrt(3) * R."""
    return math.sqrt(3.0) * contact_radius(agl, layout)


def ground_beams(position_ned: Vec3, yaw: float, agl: float, layout: EmitterLayout) -> List[Beam]:
    """
    Emitter positions and their ground contact points for the current pose.

    ``agl`` is the height of the body origin above the ground (metres, > 0).
    Raises ``ValueError`` if the emitters would be at or below the ground.
    """
    if agl - layout.mount_down <= 0.0:
        raise ValueError(f'emitters are not above the ground (agl={agl:.2f} m)')

    p_n, p_e, p_d = position_ned
    radius = contact_radius(agl, layout)
    beams: List[Beam] = []
    for phi in layout.azimuths:
        m_n, m_e = _rotate_yaw(layout.mount_radius * math.cos(phi),
                               layout.mount_radius * math.sin(phi), yaw)
        g_n, g_e = _rotate_yaw(radius * math.cos(phi), radius * math.sin(phi), yaw)
        beams.append(Beam(
            emitter_ned=(p_n + m_n, p_e + m_e, p_d + layout.mount_down),
            ground_ned=(p_n + g_n, p_e + g_e, p_d + agl),
        ))
    return beams


def ned_to_enu(v: Vec3) -> Vec3:
    """NED (N, E, D) -> ENU (E, N, U) for RViz's `map` frame."""
    return v[1], v[0], -v[2]
