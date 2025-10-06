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

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "xarm_description/urdf/xarm7_gripper.urdf"
        )
        links[9].name = "gripper_link"

        super().__init__(
            links,
            name=name,
            manufacturer="UFACTORY",
            gripper_links=links[10],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.grippers[0].tool = SE3(0, 0, 0.172)

        self.qr = np.array([0, np.pi/6, 0, np.pi/3, 0, -np.pi/4, 0])
        self.qz = np.zeros(7)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = xArm7()

    print(robot)
    print("Ready pose:", robot.qr)