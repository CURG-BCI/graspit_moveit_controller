#!/usr/bin/env python
from grasp_execution_node import GraspExecutionNode

if __name__ == '__main__':

    ge = GraspExecutionNode("ManualExecutionNode", manual_mode=True)


    # ge.robot_interface.parabola_test(0.10)
    import IPython
    IPython.embed()