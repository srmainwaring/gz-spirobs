"""
Grasping controller for 2D SpiRob
"""

import time

from gz.msgs.double_pb2 import Double
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.vector3d_pb2 import Vector3d
from gz.transport import Node


def main():
    model_name = "arm"
    tendon1_name = "tendon1"
    tendon2_name = "tendon2"

    node = Node()
    tendon1_topic = f"/model/{model_name}/tendon/{tendon1_name}/cmd_force"
    tendon2_topic = f"/model/{model_name}/tendon/{tendon2_name}/cmd_force"
    tendon1_pub = node.advertise(tendon1_topic, Double)
    tendon2_pub = node.advertise(tendon2_topic, Double)

    # durations
    packing_time = 2
    reaching_time = 4
    wrapping_time = 4
    grasping_time = 2

    # forces
    f_packing = 500
    f_wrapping = 1.5 * f_packing
    f_grasping = 1.5 * f_wrapping

    def reset(sim_time):
        cmd1 = Double()
        cmd1.data = 0
        cmd2 = Double()
        cmd2.data = 0
        tendon1_pub.publish(cmd1)
        tendon2_pub.publish(cmd2)
        print(f"[{sim_time:.2f}] Reset: tendon1: {cmd1.data:0.2f} N, tendon2: {cmd2.data:0.2f}")

    def packing(sim_time):
        cmd1 = Double()
        cmd1.data = f_packing * (sim_time / packing_time)
        cmd2 = Double()
        cmd2.data = 0
        tendon1_pub.publish(cmd1)
        tendon2_pub.publish(cmd2)
        print(f"[{sim_time:.2f}] Packing: tendon1: {cmd1.data:0.2f} N, tendon2: {cmd2.data:0.2f}")

    def reaching(sim_time):
        cmd1 = Double()
        cmd1.data = f_packing
        cmd2 = Double()
        cmd2.data = f_wrapping * ((sim_time - packing_time)/ reaching_time)
        tendon1_pub.publish(cmd1)
        tendon2_pub.publish(cmd2)
        print(f"[{sim_time:.2f}] Reaching: tendon1: {cmd1.data:0.2f} N, tendon2: {cmd2.data:0.2f}")

    def wrapping(sim_time):
        cmd1 = Double()
        cmd1.data = f_packing * ((packing_time + reaching_time + wrapping_time - sim_time)/ wrapping_time)
        cmd2 = Double()
        cmd2.data = f_wrapping
        tendon1_pub.publish(cmd1)
        tendon2_pub.publish(cmd2)
        print(f"[{sim_time:.2f}] Wrapping: tendon1: {cmd1.data:0.2f} N, tendon2: {cmd2.data:0.2f}")

    def grasping(sim_time):
        cmd1 = Double()
        cmd1.data = 0
        cmd2 = Double()
        cmd2.data = f_grasping
        tendon1_pub.publish(cmd1)
        tendon2_pub.publish(cmd2)
        print(f"[{sim_time:.2f}] Grasping: tendon1: {cmd1.data:0.2f} N, tendon2: {cmd2.data:0.2f}")

    try:
        # reset
        reset(sim_time=0)
        time.sleep(1.0)

        sim_start_time = time.time()
        while True:
            now = time.time()

            # Packing
            sim_time = now - sim_start_time
            start_time = 0
            end_time = start_time + packing_time
            if sim_time >= start_time and sim_time < end_time:
                packing(sim_time)

            # Reaching
            sim_time = now - sim_start_time
            start_time = end_time
            end_time = start_time + reaching_time
            if sim_time >= start_time and sim_time < end_time:
                reaching(sim_time)

            # Wrapping
            sim_time = now - sim_start_time
            start_time = end_time
            end_time = start_time + wrapping_time
            if sim_time >= start_time and sim_time < end_time:
                wrapping(sim_time)

            # Grasping
            sim_time = now - sim_start_time
            start_time = end_time
            end_time = start_time + grasping_time
            if sim_time >= start_time and sim_time < end_time:
                grasping(sim_time)

            time.sleep(0.2)

            sim_time = now - sim_start_time
            start_time = end_time
            if sim_time >= start_time:
                break

    except KeyboardInterrupt:
        print("Received Ctrl+C")
    finally:
        reset(sim_time=0)
        print("Exiting")


if __name__ == "__main__":
    main()
