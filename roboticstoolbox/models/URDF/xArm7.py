#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3


class xArm7(Robot):
    """
    Class that imports a xArm7 URDF model

    ``xArm7()`` is a class which imports a UFACTORY xArm7 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.xArm7()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):
        
        gripper = False

        urdf_file = "xarm_description/urdf/xarm7_gripper.urdf" if gripper else "xarm_description/urdf/xarm7_cam_1305.urdf"

        links, name, urdf_string, urdf_filepath = self.URDF_read(urdf_file)

        if gripper : links[9].name = "gripper_link"

        super().__init__(
            links,
            name=name,
            manufacturer="UFACTORY",
            gripper_links=links[10] if gripper else [],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self._joint_offsets = [
            None, None,   
            SE3.Trans(0, 0, 0),                # index 2 → no offset
            SE3.Trans(0, 0, -0.11665),         # index 3
            SE3.Trans(0, 0,  0.059),           # index 4
            SE3.Trans(0, 0, -0.11057),         # index 5
            SE3.Trans(0, 0,  0.04605108),      # index 6
            SE3.Trans(0, 0, -0.17982),         # index 7
            SE3.Trans(0, 0,  0.01209767),      # index 8
            SE3.Trans(0, 0, -0.03162),         # index 9
        ]

        if gripper : self.grippers[0].tool = SE3(0, 0, 0.172)

        self.qr = np.array([0, 0.0177, 0, 1.86751, 0, 0.72431, 0])
        self.qz = np.zeros(7)
        self.qdlim = np.pi
        self.damping_gain = np.array([10, 10, 5, 5, 5, 2, 2])
        self.friction_gain = np.array([1, 1, 1, 1, 1, 1, 1])

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

    def joints_T(self, q = None):
        """
        Returns transformed base to joints (joints_T)
        """
        if q is None:
            q = self.q    # use current robot joint state

        T_all = self.fkine_all(q)

        # apply joint offsets
        for i in range(2, 10):
            T_all[i] = T_all[i] * self._joint_offsets[i]
        
        joints_T = T_all[2:10]
        return joints_T



if __name__ == "__main__":  # pragma nocover

    robot = xArm7()

    print(robot)
    print("Ready pose:", robot.qr)