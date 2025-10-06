#!/usr/bin/env python
from roboticstoolbox.robot.Link import Link
import numpy as np
from spatialmath import SE3
from roboticstoolbox.robot.Robot import Robot


class Jaco(Robot):
    """
    Class that imports a Jaco URDF model

    ``Jaco()`` is a class which imports a KinovaGen3 robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.Jaco()
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
            "kinova_description/urdf/j2n6s300_standalone.xacro"
        )
        # n=0
        # for link in links:
        #     print(f"\nThis is the link number : {n}\n")
        #     print(link)
        #     n += 1
        
        last_link = links[8]
        gripper_links = [link for link in links if link.parent == last_link]

        j2n6s300_gripper = Link(name="j2n6s300_gripper", parent=last_link)
        links.append(j2n6s300_gripper)

        for g_link in gripper_links:
            g_link.parent = j2n6s300_gripper
        
        super().__init__(
            links,
            name=name,
            manufacturer="Kinova",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            gripper_links= j2n6s300_gripper
        )
        self.grippers[0].tool = SE3(0, 0, 0.16)

        self.qr = np.array([0, np.pi + np.pi/6, np.pi-np.pi/2, np.pi/2, 0.0, 0.0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

        j6 = next(i for i, L in enumerate(links) if L.name == "j2n6s300_link_6")
        links[j6].I = np.array([1e-4, 1e-4, 1e-4, 0, 0, 0])
        links[j6].Jm = 1e-4
        links[j6].G = 1.0
        links[j6].m = 0.30
        links[j6].r = [0.0, 0.05, 0.0] 



if __name__ == "__main__":  # pragma nocover

    robot = Jaco()
    print(robot)
