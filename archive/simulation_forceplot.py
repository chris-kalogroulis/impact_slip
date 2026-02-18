# Import some basic libraries and functions for this tutorial.
import numpy as np
import os
from pathlib import Path
import matplotlib.pyplot as plt

from pydrake.multibody.tree import RevoluteJoint, RevoluteSpring
from pydrake.multibody.tree import PrismaticJoint, LinearSpringDamper
from pydrake.common import temp_directory
from pydrake.geometry import SceneGraphConfig, StartMeshcat
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.common.value import AbstractValue # type: ignore
from pydrake.systems.primitives import LogVectorOutput
from pydrake.geometry import (
    AddCompliantHydroelasticProperties,
    AddRigidHydroelasticProperties,
    AddContactMaterial,
    ProximityProperties,
)
from pydrake.multibody.plant import ContactModel


# Start the visualizer. The cell will output an HTTP link after the execution.
# Click the link and a MeshCat tab should appear in your browser.
meshcat = StartMeshcat()

# When this notebook is run in test mode it needs to stop execution without
# user interaction. For interactive model visualization you won't normally
# need the 'loop_once' flag.
test_mode = True if "TEST_SRCDIR" in os.environ else False

class PointContactForceVectorBetweenBodies(LeafSystem):
    """Outputs summed contact force vector [Fx,Fy,Fz] (N) from point-pair contacts
    between two named bodies, expressed in world frame.
    """

    def __init__(self, plant, bodyA_name: str, bodyB_name: str):
        super().__init__()
        self._idxA = plant.GetBodyByName(bodyA_name).index()
        self._idxB = plant.GetBodyByName(bodyB_name).index()

        # Must match plant's ContactResults type exactly.
        self.DeclareAbstractInputPort(
            "contact_results",
            plant.get_contact_results_output_port().Allocate()
        )

        # 3D output: Fx, Fy, Fz
        self.DeclareVectorOutputPort(
            "force_W",
            BasicVector(3),
            self._CalcOutput
        )

    def _CalcOutput(self, context, output: BasicVector):
        cr = self.get_input_port(0).Eval(context)
        F_W = np.zeros(3)

        for i in range(cr.num_point_pair_contacts()):
            info = cr.point_pair_contact_info(i)
            a = info.bodyA_index()
            b = info.bodyB_index()

            if (a == self._idxA and b == self._idxB) or (a == self._idxB and b == self._idxA):
                f = info.contact_force()
                # Drake builds differ: sometimes f is a 3-vector, sometimes SpatialForce.
                if hasattr(f, "translational"):
                    f_vec = np.asarray(f.translational()).reshape(3,)
                else:
                    f_vec = np.asarray(f).reshape(3,)
                F_W += f_vec

        output.SetFromVector(F_W)


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
                      k=750, x0=0.40, d=10.0),
        "ankle": dict(type="revolute",  
                      k=25, q0=0.00, d=5.00)  # k [N·m/rad], d [N·m·s/rad], q0 [rad]
    }

    add_joint_springs_and_dampers(plant, joint_params)

    # ---------------------------

    plant.Finalize()
    plant_context = plant.CreateDefaultContext()

    AddDefaultVisualization(builder=builder, meshcat=meshcat)


    contact_force_sys = builder.AddSystem(
        PointContactForceVectorBetweenBodies(plant, "foot", "floor")  # <- your names
    )
    contact_force_sys.set_name("contact_force_foot_floor")

    builder.Connect(
        plant.get_contact_results_output_port(),
        contact_force_sys.GetInputPort("contact_results")
    )

    contact_force_logger = LogVectorOutput(
        contact_force_sys.GetOutputPort("force_W"),
        builder
    )
    contact_force_logger.set_name("contact_force_logger")

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

    log = diagram.GetSubsystemByName("contact_force_logger").FindLog(simulator.get_context())
    t = log.sample_times()
    F = log.data()   # shape (3, N)

    fig, axs = plt.subplots(3, 1, sharex=True)

    axs[0].plot(t, F[0, :])
    axs[0].set_ylabel("Fx [N]")
    axs[0].grid(True)
    axs[0].set_ylim((0,20))

    axs[1].plot(t, F[1, :])
    axs[1].set_ylabel("Fy [N]")
    axs[1].grid(True)
    axs[1].set_ylim((0,-50))

    axs[2].plot(t, F[2, :])
    axs[2].set_ylabel("Fz [N]")
    axs[2].set_xlabel("time [s]")
    axs[2].grid(True)
    axs[2].set_ylim((0,-150))

    plt.tight_layout()
    plt.show()



# Run the simulation with a small time step. Try gradually increasing it!
run_simulation(sim_time_step=0.0001)
