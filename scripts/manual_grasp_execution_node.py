#!/usr/bin/env python
from grasp_execution_node import GraspExecutionNode

if __name__ == '__main__':

    ge = GraspExecutionNode("ManualExecutionNode", manual_mode=True)


    import IPython
    IPython.embed()