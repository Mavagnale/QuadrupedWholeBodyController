#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import sys
import os
import math
from tf.transformations import euler_from_quaternion

def plot_from_bag(bagfile, skip_seconds=3.0):
    if not os.path.exists(bagfile):
        print(f"Bag file {bagfile} not found.")
        return

    bag = rosbag.Bag(bagfile)

    # Determine start time
    start_time = bag.get_start_time() + skip_seconds
    print(f"Skipping first {skip_seconds} seconds. Start plotting from t={start_time:.2f}s")

    # Storage
    joint_names = []
    joint_data = {}   # dict: name -> (times, positions)
    torque_data = {}  # dict: name -> (times, torques)

    # Prepare desired GRF storage (per leg)
    leg_order = ["FL", "FR", "RL", "RR"]
    desired_grf = {ln: ([], []) for ln in leg_order}  # name -> (times, vertical_forces)

    model_times, model_x, model_y, model_z = [], [], [], []
    model_roll, model_pitch, model_yaw = [], [], []

    base_contact_times, base_contact_fx = [], []

    tank_times, tank_energy = [], []
    alpha_times, alpha_value = [], []

    print("Reading bag...")

    for topic, msg, t in bag.read_messages(topics=[
        "/anymal/joint_states",
        "/gazebo/model_states",
        "/anymal/joint_effort_group_controller/command",
        "/anymal/desired_ground_reaction_forces",
        "/base_contact_sensor",
        "/anymal/tank_energy",
        "/anymal/alpha"
    ]):
        timestamp = t.to_sec()
        if timestamp < start_time:
            continue

        if topic == "/anymal/joint_states":
            if not joint_names:  # initialize once
                joint_names = msg.name
                for j in joint_names:
                    joint_data[j] = ([], [])
                    torque_data[j] = ([], [])

            for idx, name in enumerate(msg.name):
                times, positions = joint_data[name]
                times.append(timestamp - start_time)
                positions.append(msg.position[idx])

        elif topic == "/gazebo/model_states":
            if "anymalModel" in msg.name:
                idx = msg.name.index("anymalModel")
                pose = msg.pose[idx]
                model_times.append(timestamp - start_time)
                model_x.append(pose.position.x)
                model_y.append(pose.position.y)
                model_z.append(pose.position.z)

                # orientation → roll, pitch, yaw
                q = pose.orientation
                quat = [q.x, q.y, q.z, q.w]
                roll, pitch, yaw = euler_from_quaternion(quat)
                model_roll.append(roll)
                model_pitch.append(pitch)
                model_yaw.append(yaw)

        elif topic == "/anymal/joint_effort_group_controller/command":
            if joint_names:
                for idx, name in enumerate(joint_names):
                    times, torques = torque_data[name]
                    times.append(timestamp - start_time)
                    torques.append(msg.data[idx])

        elif topic == "/anymal/desired_ground_reaction_forces":
            # Support multiple possible message shapes:
            # - msg.wrenches: list of geometry_msgs/Wrench (each has .force.x/.y/.z)
            # - msg.data: flat list/array with chunked values (e.g. 12 values -> 4 legs * 3 components)
            leg_forces = []  # list of (fx,fy,fz) tuples in the same order as leg_order
            data = list(msg.data)
            nlegs = len(leg_order)
            if len(data) % nlegs == 0:
                chunk = len(data) // nlegs
                for i in range(nlegs):
                    chunk_vals = data[i*chunk:(i+1)*chunk]
                    # prefer (fx,fy,fz) if chunk >=3, else take what is available and pad with zeros
                    fx = chunk_vals[0] if len(chunk_vals) > 0 else 0.0
                    fy = chunk_vals[1] if len(chunk_vals) > 1 else 0.0
                    fz = chunk_vals[2] if len(chunk_vals) > 2 else 0.0
                    leg_forces.append((fx, fy, fz))

            # Store vertical force (z) per leg (assume order matches leg_order)
            for i, ln in enumerate(leg_order):
                if i < len(leg_forces):
                    _, _, fz = leg_forces[i]
                else:
                    fz = 0.0
                times, vals = desired_grf[ln]
                times.append(timestamp - start_time)
                vals.append(fz)

        elif topic == "/base_contact_sensor":
            # Extract the same component accessed in C++ as:
            # contactMsg.states[0].wrenches[0].force.x
            fx = 0.0
            if hasattr(msg, "states") and len(msg.states) > 0:
                state0 = msg.states[0]
                # ContactState typically has 'wrenches' (list of geometry_msgs/Wrench)
                if hasattr(state0, "wrenches") and len(state0.wrenches) > 0:
                    fx = getattr(state0.wrenches[0].force, "x", 0.0)
                # fallback: some message variants store a single 'wrench'
                elif hasattr(state0, "wrench") and hasattr(state0.wrench, "force"):
                    fx = getattr(state0.wrench.force, "x", 0.0)
            base_contact_times.append(timestamp - start_time)
            base_contact_fx.append(abs(fx))

        elif topic == "/anymal/tank_energy":
            # std_msgs/Float64 -> msg.data
            tank_times.append(timestamp - start_time)
            tank_energy.append(msg.data if hasattr(msg, "data") else 0.0)

        elif topic == "/anymal/alpha":
            alpha_times.append(timestamp - start_time)
            alpha_value.append(msg.data if hasattr(msg, "data") else 0.0)


    bag.close()

    # Define legs and their joints
    legs = {
        "FL": ["LF_HAA", "LF_HFE", "LF_KFE"],
        "FR": ["RF_HAA", "RF_HFE", "RF_KFE"],
        "RL": ["LH_HAA", "LH_HFE", "LH_KFE"],
        "RR": ["RH_HAA", "RH_HFE", "RH_KFE"]
    }

    # --- Plot 1: Joint positions by leg ---
    fig1, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig1.suptitle("Joint Positions by Leg")
    axes = axes.flatten()

    for i, (leg_name, joints) in enumerate(legs.items()):
        for jname in joints:
            times, positions = joint_data[jname]
            axes[i].plot(times, positions, label=jname)
        axes[i].set_title(leg_name)
        axes[i].set_ylabel("rad")
        axes[i].legend(fontsize="small")
        axes[i].grid(True)
    axes[-1].set_xlabel("Time [s] (relative)")

    # --- Plot 2: Robot state ---
    fig2, axes2 = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig2.suptitle("Robot Base State Over Time")

    labels = ["x [m]", "y [m]", "z [m]", "roll [deg]", "pitch [deg]", "yaw [deg]"]
    data = [model_x, model_y, model_z,
            [math.degrees(r) for r in model_roll],
            [math.degrees(p) for p in model_pitch],
            [math.degrees(y) for y in model_yaw]]

    for i, (lab, arr) in enumerate(zip(labels, data)):
        axes2[i].plot(model_times, arr, label=lab)
        axes2[i].set_ylabel(lab)
        axes2[i].grid(True)
    axes2[-1].set_xlabel("Time [s] (relative)")

    # --- Plot 3: Joint torques by leg ---
    fig3, axes3 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig3.suptitle("Commanded Joint Torques by Leg")
    axes3 = axes3.flatten()

    for i, (leg_name, joints) in enumerate(legs.items()):
        for jname in joints:
            times, torques = torque_data[jname]
            axes3[i].plot(times, torques, label=jname)
        axes3[i].set_title(leg_name)
        axes3[i].set_ylabel("Nm")
        axes3[i].legend(fontsize="small")
        axes3[i].grid(True)
    axes3[-1].set_xlabel("Time [s] (relative)")

    # --- Plot 4: Desired ground reaction forces (vertical) by leg ---
    # Only plot if we have any data
    have_grf = any(len(desired_grf[ln][0]) > 0 for ln in leg_order)
    if have_grf:
        fig4, axes4 = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
        fig4.suptitle("Desired Ground Reaction Forces (vertical) by Leg")
        axes4 = axes4.flatten()
        for i, ln in enumerate(leg_order):
            times, vz = desired_grf[ln]
            axes4[i].plot(times, vz, label=f"{ln} Fz")
            axes4[i].set_title(ln)
            axes4[i].set_ylabel("Fz [N]")
            axes4[i].grid(True)
            axes4[i].legend(fontsize="small")
        axes4[-1].set_xlabel("Time [s] (relative)")

    # --- Plot 5: Base contact sensor fx (states[0].wrenches[0].force.x) ---
    if len(base_contact_times) > 0:
        fig5, ax5 = plt.subplots(1, 1, figsize=(10, 3.5))
        fig5.suptitle("Base Contact Sensor: |fx|")
        ax5.plot(base_contact_times, base_contact_fx, label="base_contact_fx")
        ax5.set_ylabel("Force X [N]")
        ax5.set_xlabel("Time [s] (relative)")
        ax5.grid(True)
        ax5.legend(fontsize="small")

    # --- Plot 6: Tank energy and alpha (std_msgs/Float64) ---
    if len(tank_times) > 0 or len(alpha_times) > 0:
        fig6, axes6 = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        fig6.suptitle("Tank Energy and Alpha")
        if len(tank_times) > 0:
            axes6[0].plot(tank_times, tank_energy, label="tank_energy", color="tab:blue")
        axes6[0].set_ylabel("Energy")
        axes6[0].grid(True)
        axes6[0].legend(fontsize="small")

        if len(alpha_times) > 0:
            axes6[1].plot(alpha_times, alpha_value, label="alpha", color="tab:orange")
        axes6[1].set_ylabel("Alpha")
        axes6[1].set_xlabel("Time [s] (relative)")
        axes6[1].grid(True)
        axes6[1].legend(fontsize="small")

    # Save all figures as PDFs
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rosbags", "plots"))
    os.makedirs(output_dir, exist_ok=True)
    try:
        fig1.savefig(os.path.join(output_dir, "joint_positions_by_leg.pdf"), bbox_inches="tight")
        fig2.savefig(os.path.join(output_dir, "base_state_over_time.pdf"), bbox_inches="tight")
        fig3.savefig(os.path.join(output_dir, "joint_torques_by_leg.pdf"), bbox_inches="tight")
        if "fig4" in locals():
            fig4.savefig(os.path.join(output_dir, "desired_grf_by_leg.pdf"), bbox_inches="tight")
        if "fig5" in locals():
            fig5.savefig(os.path.join(output_dir, "base_contact_fx.pdf"), bbox_inches="tight")
        if "fig6" in locals():
            fig6.savefig(os.path.join(output_dir, "tank_energy_and_alpha.pdf"), bbox_inches="tight")
        print(f"Saved plots to: {output_dir}")
    except Exception as e:
        print(f"Error saving plots: {e}")

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_bag.py <bagfile> [skip_seconds]")
        sys.exit(1)

    bagfile = sys.argv[1]
    skip_seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    plot_from_bag(bagfile, skip_seconds)