# Import some basic libraries and functions for this tutorial.
import numpy as np
import os
from pathlib import Path

from pydrake.multibody.tree import RevoluteJoint, RevoluteSpring
from pydrake.multibody.tree import PrismaticJoint, LinearSpringDamper
from pydrake.common import temp_directory
from pydrake.geometry import SceneGraphConfig, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer

# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# When this notebook is run in test mode it needs to stop execution without
# user interaction. For interactive model visualization you won't normally
# need the 'loop_once' flag.
test_mode = True if "TEST_SRCDIR" in os.environ else False

def add_joint_springs_and_dampers(plant: MultibodyPlant, joint_params: dict):
    """
    Adds:
      - default viscous damping on the joint (Drake's built-in joint damping term)
      - a spring+damper force element about/along that joint (LinearSpringDamper)
    """
    for name, p in joint_params.items():

        j = plant.GetJointByName(name)

        if isinstance(j, RevoluteJoint):

            j.set_default_damping(p["d"])

            plant.AddForceElement(
                RevoluteSpring(
                    joint=j,
                    nominal_angle=p["q0"],   # [rad]
                    stiffness=p["k"]           # [N·m/rad]
                )
            )

        elif isinstance(j, PrismaticJoint):
            A = plant.GetBodyByName(p["A"])
            B = plant.GetBodyByName(p["B"])

            # Choose axis direction in *bodyB frame* (this is a guess; adjust!)
            axis_B = np.array([1., 0., 0.])   # if prismatic axis is +x in B frame
            d = 0.10                          # must stay > 0 always

            p_AP = np.array([0., 0., 0.])     # on A (A frame)
            p_BQ = d * axis_B                 # on B (B frame)

            plant.AddForceElement(
                LinearSpringDamper(
                    bodyA=A, p_AP=p_AP,
                    bodyB=B, p_BQ=p_BQ,
                    free_length=d + p["x0"],  # "rest" at x0
                    stiffness=p["k"],
                    damping=p["d"]
                )
            )

        else:
            raise TypeError(f"Joint '{name}' must be Revolute or Prismatic (got {type(j)}).")


def create_scene(sim_time_step):
    # Clean up the Meshcat instance.
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder= DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step)
    parser = Parser(builder)

    # Path to the URDF (in the same directory as this script)
    urdf_path = Path("robot.urdf")
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path.resolve()}")

    parser.AddModels(str(urdf_path))

        # ---------------------------
    # Add torsional springs + dampers (BEFORE Finalize)
    # ---------------------------
    joint_params = {
        "rail":  dict(type="prismatic", A="base", B="prismatic_coupler",      
                      k=00.0, x0=0.00, d=0.00), # k [N/m], d [N·s/m], q0 [m]
        "thigh": dict(type="prismatic", A="prismatic_coupler", B="upper_leg", 
                      k=00.0, x0=0.00, d=0.00),
        "knee":  dict(type="prismatic", A="upper_leg", B="lower_leg",         
                      k=750, x0=0.40, d=0.0),
        "ankle": dict(type="revolute",  
                      k=25, q0=0.00, d=0.00)  # k [N·m/rad], d [N·m·s/rad], q0 [rad]
    }

    add_joint_springs_and_dampers(plant, joint_params)

    # ---------------------------

    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    return diagram

def initialize_simulation(diagram):
    simulator = Simulator(diagram)

    context = simulator.get_mutable_context()

    # Get the plant from the diagram
    plant = diagram.GetSubsystemByName("plant")
    plant_context = plant.GetMyMutableContextFromRoot(context)

    rail = plant.GetJointByName("rail")
    rail.set_translation(plant_context, 0.6)

    thigh = plant.GetJointByName("thigh")
    thigh.set_translation(plant_context, -0.3)

    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    return simulator

def run_simulation(sim_time_step):
    diagram = create_scene(sim_time_step)
    simulator = initialize_simulation(diagram)
    meshcat.StartRecording()
    finish_time = 0.1 if test_mode else 5.0
    simulator.AdvanceTo(finish_time)
    meshcat.PublishRecording()

# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0001)