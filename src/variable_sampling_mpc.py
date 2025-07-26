import numpy as np
import toml
from mujoco_lib.ironcub_mujoco_simulator import MujocoConfig, MujocoSim
import flight_ctrl_lib.bindings as flightCtrl
from momentum_based_mpc.bindingsMPC import VariableSamplingMPC
import bipedal_locomotion_framework.bindings as blf
import time
from pathlib import Path
import scipy.io
import mujoco

SAVE_DATA = True
MAX_TRAJECTORY_LENGTH = 20000

if __name__ == "__main__":

    rf = flightCtrl.ResourceFinder()

    toml_file_path = Path(__file__).parents[0] / "config/configMujoco.toml"

    configSim = MujocoConfig(toml.load(toml_file_path))

    robot_config_file = Path(__file__).parents[0] / "config/robot.toml"
    configSim.config["robot_config_file"] = str(robot_config_file)

    xml_path = rf.findFileByName("iRonCub-Mk3_Mujoco/iRonCub.xml")
    configSim.config["mujoco_model_path"] = str(xml_path)

    run_viz = True
    n_iter = 10000
    sim = MujocoSim(configSim, run_visulaization=run_viz)

    flightCtrl.useSystemClock()
    param_handler = blf.parameters_handler.YarpParametersHandler()
    rf = flightCtrl.ResourceFinder()
    configFileFullPath = str(Path(__file__).parents[0] / "config/vs_mcp_config.xml")
    print("configFileFullPath:", configFileFullPath)
    if not flightCtrl.readXMLFile(configFileFullPath, param_handler):
        raise RuntimeError("Failed to read the configuration file")
    
    param_handler_mpc = param_handler.get_group("VS_MPC_CONFIG")

    MPCPeriod = param_handler_mpc.get_parameter_float("periodMPC")
    sim.set_alpha_LP(MPCPeriod, 3)
    periodSim = sim.get_simulation_timestep()
    n_steps = int(MPCPeriod / periodSim)

    initial_throttle = np.zeros(4)
    estimated_thrust = sim.get_estimated_thrust()
    estimated_thrust_dot = np.zeros(4)

    qpInput = flightCtrl.QPInput()
    qpInput.setRobot(sim.robot)
    qpInput.setRobotReference(sim.robot)
    qpInput.setEmptyVectorsCollectionServer()
    qpInput.setThrottleMPC(initial_throttle)
    qpInput.setEmptyJetModel()
    qpInput.setThrustDesMPC(estimated_thrust)
    qpInput.setThrustDotDesMPC(np.zeros(4))
    qpInput.setEstimatedThrustDot(estimated_thrust_dot)
    qpInput.setOutputQPJointsPosition(sim.get_joint_positions())
    measured_joint_positions = sim.get_joint_positions()
    sim.set_joint_positions(measured_joint_positions)
    for _ in range(2 * 200):
        sim.step(n_steps)

    joint_pos_init = sim.get_joint_positions()

    sim.update_robot_state()

    variable_sampling_mpc = VariableSamplingMPC()

    if not variable_sampling_mpc.configure(param_handler_mpc, qpInput):
        raise RuntimeError("Failed to initialize the MPC")

    mom_dot = []
    time_ctrl = 0.0
    CoMPosition = np.array(sim.robot.getPositionCoM())
    CoMPosition_desired = np.array(qpInput.getPosCoMReference())
    CoMPosition_render = []
    CoMPosition_ref_render = []
    estimated_thrust = []
    estimated_thrust_dot = []
    thrust_desired = []
    thrust_desired_dot = []
    base_position = []
    base_orientation = []
    base_orientation_desired = np.array(qpInput.getRPYReference())
    base_lin_vel = []
    base_ang_vel = []
    base_lin_vel_filtered = []
    base_ang_vel_filtered = []
    linear_momentum = np.array(sim.robot.getMomentum(True)[:3])
    angular_momentum = np.array(sim.robot.getMomentum(True)[3:])
    momentum_reference = np.array(qpInput.getMomentumReference())
    alpha_gravity = []
    joint_pos_meas = []
    joint_pos_ref = []
    throttle_saved = []
    time_controller = []
    time_MPC = []
    max_geoms_needed = sim.model.ngeom + (MAX_TRAJECTORY_LENGTH if MAX_TRAJECTORY_LENGTH > 1 else 0) + 10
    render = mujoco.Renderer(sim.model, width=1920, height=1088)
    frames = []
    counter = 0
    counter_video = 0
    time_sum = 0

    while sim.is_running():
        sim.update_robot_state()
        estim_thrust_dot = sim.get_estimated_thrust_dot()
        qpInput.setEstimatedThrustDot(estim_thrust_dot)
        tic = time.time()
        variable_sampling_mpc.update(qpInput)
        variable_sampling_mpc.solveMPC()
        toc = time.time()
        time_MPC.append(toc - tic)
        time_sum += toc - tic
        if counter == 200:
            print("average time MPC: ", time_sum / counter, ", time elapsed: " , time_ctrl)
            time_sum = 0
            counter = 0
        counter += 1

        if (toc - tic) > MPCPeriod:
            print("MPC exceeded the period by:", (toc - tic) - MPCPeriod)
        desired_thrust = variable_sampling_mpc.getThrustReference()
        desired_thrust_dot = variable_sampling_mpc.getThrustDotReference()
        throttle_command = variable_sampling_mpc.getThrottleReference()
        joint_positions_reference = variable_sampling_mpc.getJointsReferencePosition()
        qpInput.setThrottleMPC(throttle_command)
        qpInput.setThrustDesMPC(desired_thrust)
        qpInput.setThrustDotDesMPC(desired_thrust_dot)
        qpInput.setOutputQPJointsPosition(joint_positions_reference)
        if not sim._use_nn_jet_model:
            sim.set_thrust(desired_thrust)
        sim.set_joint_positions(joint_positions_reference)
        sim.set_throttle(throttle_command)
        time_ctrl += MPCPeriod
        if SAVE_DATA:
            CoMPosition = np.vstack((CoMPosition, np.array(sim.robot.getPositionCoM())))
            CoMPosition_desired = np.vstack((CoMPosition_desired, np.array(qpInput.getPosCoMReference())))
            estimated_thrust.append(sim.get_estimated_thrust())
            estimated_thrust_dot.append(sim.get_estimated_thrust_dot())
            thrust_desired.append(desired_thrust)
            thrust_desired_dot.append(desired_thrust_dot)
            base_position.append(sim.robot.getBasePosition())
            base_orientation.append(sim.robot.getBaseOrientation())
            base_orientation_desired = np.vstack((base_orientation_desired, np.array(qpInput.getRPYReference())))
            base_lin_vel.append(sim.get_base_velocity())
            base_ang_vel.append(sim.get_base_angular_velocity())
            base_lin_vel_filtered.append(sim._base_lin_vel_filtered)
            base_ang_vel_filtered.append(sim._base_ang_vel_filtered)
            linear_momentum = np.vstack((linear_momentum, sim.robot.getMomentum(True)[:3]))
            angular_momentum = np.vstack((angular_momentum, sim.robot.getMomentum(True)[3:]))
            momentum_reference = np.vstack((momentum_reference, np.array(qpInput.getMomentumReference())))
            alpha_gravity.append(qpInput.getAlphaGravity())
            joint_pos_meas.append(sim.get_joint_positions())
            joint_pos_ref.append(joint_positions_reference)
            time_controller.append(time_ctrl)
            throttle_saved.append(throttle_command)
            mom_dot.append(sim.robot.getMatrixAmomJets() @ sim.get_estimated_thrust())

        sim.step(n_steps)

if SAVE_DATA:
    data = {
        "CoMPosition": CoMPosition,
        "CoMPosition_desired": CoMPosition_desired,
        "base_orientation_desired": base_orientation_desired,
        "base_position": base_position,
        "base_orientation": base_orientation,
        "base_lin_vel": np.array(base_lin_vel),
        "base_ang_vel": np.array(base_ang_vel),
        "base_lin_vel_filtered": np.array(base_lin_vel_filtered),
        "base_ang_vel_filtered": np.array(base_ang_vel_filtered),
        "joints_pos_meas": np.array(joint_pos_meas),
        "joints_pos_ref": np.array(joint_pos_ref),
        "linear_momentum": np.array(linear_momentum),
        "angular_momentum": np.array(angular_momentum),
        "momentum_reference": momentum_reference,
        "estimated_thrust": np.array(estimated_thrust),
        "estimated_thrust_dot": np.array(estimated_thrust_dot),
        "thrust_desired": np.array(thrust_desired),
        "thrust_desired_dot": np.array(thrust_desired_dot),
        "alpha_gravity": alpha_gravity,
        "throttle": np.array(throttle_saved),
        "mom_dot": np.array(mom_dot),
        "time_controller": time_controller,
        "time_MPC": np.array(time_MPC)
    }

    # get the current date and time
    now = time.localtime()
    current_time = time.strftime("%Y-%m-%d_%H-%M-%S", now)

    scipy.io.savemat(current_time + ".mat", data)
