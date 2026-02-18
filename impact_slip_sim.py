"""
Refactor of your script into:
  - create_scene(...): builds the Diagram / Scene (plant + viz), adds joint elements, finalizes, returns (diagram, meshcat)
  - simulate_impact_sliptest(...): runs the sim using a scene created by create_scene

Changes per request:
  - kept logic the same (same plant build, finalize, viz, initial joint set, recording)
  - removed default values (you must pass everything you care about)
  - simulate_impact_sliptest now calls create_scene
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional, Tuple

import numpy as np

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, ContactModel
from pydrake.multibody.tree import RevoluteJoint, PrismaticJoint, RevoluteSpring, LinearSpringDamper
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RigidTransform, RollPitchYaw


# -----------------------------
# Joint elements
# -----------------------------
def add_joint_springs_and_dampers(plant: MultibodyPlant, joint_params: Dict[str, Dict[str, Any]]):
    """
    For each joint in joint_params:
      - set default joint viscous damping (Drake's built-in joint damping term)
      - add explicit spring element:
          Revolute -> RevoluteSpring(nominal_angle=q0, stiffness=k)
          Prismatic -> LinearSpringDamper(...) with rest at x0 and damping d

    Expected params:
      Revolute:
        {"k": Nm_per_rad, "d": Nms_per_rad, "q0": rad}

      Prismatic:
        {"A": "bodyA_name", "B": "bodyB_name", "k": N_per_m, "d": Ns_per_m, "x0": m,
         "axis_B": [x,y,z] (optional, default [1,0,0]),
         "anchor_distance": d0 (optional, default 0.10)}
    """
    for joint_name, p in joint_params.items():
        j = plant.GetJointByName(joint_name)

        if isinstance(j, RevoluteJoint):
            j.set_default_damping(float(p.get("d", 0.0)))
            plant.AddForceElement(
                RevoluteSpring(
                    joint=j,
                    nominal_angle=float(p.get("q0", 0.0)),
                    stiffness=float(p.get("k", 0.0)),
                )
            )

        elif isinstance(j, PrismaticJoint):
            A = plant.GetBodyByName(p["A"])
            B = plant.GetBodyByName(p["B"])

            axis_B = np.asarray(p.get("axis_B", [1.0, 0.0, 0.0]), dtype=float).reshape(3,)
            axis_B /= np.linalg.norm(axis_B) + 1e-12

            d0 = float(p.get("anchor_distance", 0.10))
            if d0 <= 0:
                raise ValueError(f"anchor_distance must be > 0 for joint '{joint_name}'")

            p_AP = np.array([0.0, 0.0, 0.0])  # in A frame
            p_BQ = d0 * axis_B               # in B frame

            plant.AddForceElement(
                LinearSpringDamper(
                    bodyA=A, p_AP=p_AP,
                    bodyB=B, p_BQ=p_BQ,
                    free_length=d0 + float(p.get("x0", 0.0)),
                    stiffness=float(p.get("k", 0.0)),
                    damping=float(p.get("d", 0.0)),
                )
            )

            j.set_default_damping(float(p.get("d", 0.0)))

        else:
            raise TypeError(f"Joint '{joint_name}' must be Revolute or Prismatic (got {type(j)}).")


# -----------------------------
# Build scene / diagram
# -----------------------------
def create_scene(
    *,
    urdf_path: str,
    terr_path: str,
    slope: float,
    sim_time_step: float,
    joint_params: Dict[str, Dict[str, Any]],
    contact_model: ContactModel,
    meshcat=None,
):
    """
    Builds the diagram/scene and returns:
      diagram, meshcat

    Notes:
      - Adds springs/dampers BEFORE Finalize (same as your script).
      - Sets the contact model, finalizes, and adds default visualization.
    """
    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path.resolve()}")

    if meshcat is None:
        meshcat = StartMeshcat()
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)

    parser = Parser(plant)
    parser.AddModels(str(urdf_path))
    terrain = parser.AddModels(str(terr_path))[0]

    slope = np.deg2rad(slope)
    length = 1.5

    y_offset = 0.45 - (length - length * np.cos(slope))/2
    z_offset = 0.15 - (length * np.sin(slope))/2

    T = RigidTransform(RollPitchYaw(0, slope, np.pi/2), [0, y_offset, z_offset])

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("terrain", terrain),
        T
    )

    # Add springs/dampers BEFORE Finalize
    add_joint_springs_and_dampers(plant, joint_params)

    plant.set_contact_model(contact_model)
    plant.Finalize()

    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    diagram = builder.Build()
    return diagram, meshcat


# -----------------------------
# Run simulation
# -----------------------------
def simulate_impact_sliptest(
    *,
    urdf_path: str,
    terr_path: str,
    sim_time_step: float,
    duration: float,
    realtime_rate: float,
    rail_q0: float,
    rail_v0: float,
    thigh_q0: float,
    thigh_v0: float,
    slope: float,
    joint_params: Dict[str, Dict[str, Any]],
    contact_model: ContactModel,
    test_mode: Optional[bool],
):
    """
    Returns:
      simulator, diagram
    """

    # Preserve your "auto test-mode" behavior, but you must explicitly pass test_mode (or None).
    if test_mode is None:
        test_mode = True if "TEST_SRCDIR" in os.environ else False
    if test_mode:
        duration = min(duration, 0.1)

    diagram, meshcat = create_scene(
        urdf_path=urdf_path,
        terr_path=terr_path,
        slope=slope,
        sim_time_step=sim_time_step,
        joint_params=joint_params,
        contact_model=contact_model,
    )

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_sub = diagram.GetSubsystemByName("plant")
    plant_context = plant_sub.GetMyMutableContextFromRoot(context)

    # Set initial translations for the two joints you care about
    rail = plant_sub.GetJointByName("rail")
    rail.set_translation(plant_context, rail_q0)
    rail.set_translation_rate(plant_context, rail_v0)

    thigh = plant_sub.GetJointByName("thigh")
    thigh.set_translation(plant_context, thigh_q0)
    thigh.set_translation_rate(plant_context, thigh_v0)

    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate)

    meshcat.StartRecording()
    simulator.AdvanceTo(duration)
    meshcat.PublishRecording()

    return simulator, diagram


# -----------------------------
# Example call (explicit params)
# -----------------------------
if __name__ == "__main__":
    joint_params = {
        "rail":  dict(A="base", B="prismatic_coupler",
                      k=0.0, x0=0.00, d=0.00),

        "thigh": dict(A="prismatic_coupler", B="upper_leg",
                      k=0.0, x0=0.00, d=0.00),

        "knee":  dict(A="upper_leg", B="lower_leg",
                      k=800.0, x0=0.40, d=10.0),

        "ankle": dict(k=10.0, q0=0.00, d=5.00),
    }

    simulate_impact_sliptest(
        urdf_path="impact_slip_rig.urdf",
        terr_path="terr_geom.urdf",
        sim_time_step=1e-4,
        duration=3.0,
        realtime_rate=1.0,
        rail_q0=0.65,
        rail_v0=0.0,
        thigh_q0=-0.55,
        thigh_v0=0.10,
        slope=40,
        joint_params=joint_params,
        contact_model=ContactModel.kHydroelasticWithFallback,
        test_mode=None,
    )
