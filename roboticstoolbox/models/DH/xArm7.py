#!/usr/bin/env python3

import numpy as np
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath.base import transl




class xArm7(DHRobot):
    """
    UFACTORY xArm7 model using Modified DH (MDH) parameters.

    Sources of numbers:
    - MDH table (d, a in mm; alpha in rad) provided by user.
    - Dynamics (mass, COM origin, inertia tensor components) provided by user.
    - Joint limits provided by user.

    Notes:
    - Units are SI (m, kg, kg*m^2).
    - Inertia ordering for RTB is assumed to follow the same pattern as the Panda example:
      I = [Ixx, Iyy, Izz, Ixy, Iyz, Ixz]
    - URDF damping/friction are not mapped (cannot confirm mapping without RTB docs/version).
    """

    def __init__(self):
        mm = 1e-3

        # --- Joint limits (rad), provided by user ---
        qlim = [
            (-2*np.pi,  2*np.pi),          # Joint 1
            (-2.059,     2.0944),          # Joint 2
            (-2*np.pi,  2*np.pi),          # Joint 3
            (-0.19198,   3.927),           # Joint 4
            (-2*np.pi,  2*np.pi),          # Joint 5
            (-1.69297,   np.pi),           # Joint 6
            (-2*np.pi,  2*np.pi),          # Joint 7
        ]

        # --- MDH parameters from your table (convert mm -> m) ---
        # Joint i: (a, d, alpha, offset)
        mdh = [
            (0.0 * mm,     267.0 * mm,   0.0,          0.0),          # J1
            (0.0 * mm,       0.0 * mm,  -np.pi/2,      0.0),          # J2
            (0.0 * mm,     293.0 * mm,   np.pi/2,      0.0),          # J3
            (52.5 * mm,      0.0 * mm,   np.pi/2,      0.0),          # J4
            (77.5 * mm,    342.5 * mm,   np.pi/2,      0.0),          # J5
            (0.0 * mm,       0.0 * mm,   np.pi/2,      0.0),          # J6
            (76.0 * mm,     97.0 * mm,  -np.pi/2,      0.0),          # J7
        ]

        # --- Dynamics from your data (already SI units) ---
        dyn = [
            # link1
            dict(
                m=2.459,
                r=[0.00013, 0.0301, -0.012],
                I=[0.005795, 0.004969, 0.003428, 1.078e-05, -0.000911, 2.63e-05],
            ),
            # link2
            dict(
                m=1.916,
                r=[0.0002, -0.12964, 0.01692],
                I=[0.0097184, 0.0038705, 0.007672145, 1.0e-06, -0.0032, 4.83e-06],
            ),
            # link3
            dict(
                m=1.6854,
                r=[0.04676, -0.02526, -0.00746],
                I=[0.00315878, 0.002682, 0.0027105, 0.00031443, 0.0003469, -0.00058658],
            ),
            # link4
            dict(
                m=1.774,
                r=[0.07066, -0.11664, 0.0117],
                I=[0.005967, 0.00363897, 0.005509226, 0.00138232, -0.0017806, 0.00088544],
            ),
            # link5
            dict(
                m=1.357,
                r=[-0.00031, 0.01558, -0.0253],
                I=[0.005396, 0.0050232, 0.00138734, 1.5312e-05, -0.00020544, 6.7e-07],
            ),
            # link6
            dict(
                m=1.362,
                r=[0.065, 0.03336, 0.02131],
                I=[0.0015057, 0.0019297, 0.0024, -0.000496735, 0.00015, 0.00029968],
            ),
            # link7
            dict(
                m=0.17,
                r=[0.0, -0.00677, -0.01098],
                I=[9.3e-05, 5.87e-05, 0.000132, -0.0, -3.6e-06, -0.0],
            ),
            # F/T sensor link
            dict(
                m=0.445,
                r=[0.0, 0.0, 0.02553],
                I=[2.839e-04, 2.839e-04, 3.139e-04, 0.0, 0.0, 0.0],
            ),
            # camera link
            dict(
                m=0.04,
                r=[0.02298, 0.0, 0.02663],
                I=[2.66e-05, 1.24e-05, 3.90e-05, 0.0, 0.0, 0.0],
            ),
        ]

        # --- Integration of F/T sensor and camera dynamics data to link 7 ---
        def getInertiaMatrix(I):
            return np.array([
                [I[0], I[3], I[5]],
                [I[3], I[1], I[4]],
                [I[5], I[4], I[2]],
            ])
        def parallel_axis(I, m, d):
            d = np.asarray(d)
            I = getInertiaMatrix(I)
            return I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

        # Link7
        m_7 = dyn[7]["m"]
        r_7 = np.array(dyn[6]["r"])
        I_7 = np.array(dyn[6]["I"])

        # F/T sensor
        L_ft = 0.05860
        m_ft = dyn[7]["m"]
        r_ft = np.array(dyn[7]["r"])
        I_ft = np.array(dyn[7]["I"])

        # Camera
        L_cam = 0.003
        m_cam = dyn[8]["m"]
        r_cam = np.array(dyn[8]["r"])
        I_cam = np.array(dyn[8]["I"])

        # Equivalent values for link7
        m_eq = m_7 + m_ft + m_cam
        r_eq = (m_7*r_7 + m_ft*r_ft + m_cam*r_cam)/m_eq
        I_eq = (parallel_axis(I_7,   m_7,   r_7   - r_eq) +
                parallel_axis(I_ft, m_ft, r_ft  - r_eq) +
                parallel_axis(I_cam, m_cam,  r_cam - r_eq))
        I_eq = [I_eq[0,0], I_eq[1,1], I_eq[2,2], I_eq[0,1], I_eq[1,2], I_eq[0,2]]
        # Updating link7 dynamics data
        dyn[6]["m"] = m_eq
        dyn[6]["r"] = r_eq
        dyn[6]["I"] = I_eq

        # --- Build links ---
        L = []
        for i in range(7):
            a_i, d_i, alpha_i, offset_i = mdh[i]
            di = dyn[i]
            L.append(
                RevoluteMDH(
                    a=a_i,
                    d=d_i,
                    alpha=alpha_i,
                    offset=offset_i,
                    qlim=np.array(qlim[i]),
                    m=di["m"],
                    r=di["r"],
                    I=di["I"],
                    G=1,
                )
            )


        super().__init__(
            L,
            name="xArm7",
            manufacturer="UFACTORY",
            # tool can be set here if you have a measured tool/flange transform
            tool=transl(0, 0, (L_ft+L_cam)),
        )
        # Tool mass and COM for computing GW
        self.toolMass = 0.0
        self.toolCOM = np.zeros(3)
        # Base mass and COM for computing GW
        self.baseMass = 2.9710
        self.baseCOM = np.array([0.0, 0.0, 0.05])

        # convenient default configurations
        self.qz = np.zeros(7)
        # "qr" as mid-range of limits (safe default)
        self.qr = np.array([(lo + hi) / 2 for (lo, hi) in qlim])

        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qr", self.qr)
        self.qdlim = np.pi
        self.tau_lim = np.array([50.0, 50.0, 30.0, 30.0, 30.0, 20.0, 20.0])
        self.tau_db = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])


if __name__ == "__main__":
    robot = xArm7()
    print(robot)