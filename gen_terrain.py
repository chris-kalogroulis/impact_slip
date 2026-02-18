import xml.etree.ElementTree as ET
import numpy as np


def prettify(elem, level=0):
    indent = "  "
    i = "\n" + level * indent
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + indent
        for child in elem:
            prettify(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


class URDFBuilder:
    def __init__(self, name="generated_robot", drake_props=None):
        # Register drake namespace so output uses drake: prefix
        self.DRAKE_NS = "drake"
        self.DRAKE_URI = "http://drake.mit.edu"
        ET.register_namespace(self.DRAKE_NS, self.DRAKE_URI)

        self.robot = ET.Element("robot", name=name)

        # Default proximity properties (override by passing drake_props dict)
        self.drake_props = drake_props or {
            "hydroelastic_type": "compliant",   # "compliant" or "rigid"
            "hydroelastic_modulus": "5e7",      # Pa
            "hunt_crossley_dissipation": "1.0", # s/m
            "resolution_hint": "0.01",          # m
            "mu_static": "1.0",
            "mu_dynamic": "0.8",
        }

    def _add_drake_proximity_properties(self, collision_elem):
        drake_tag = lambda t: f"{{{self.DRAKE_URI}}}{t}"
        prox = ET.SubElement(collision_elem, drake_tag("proximity_properties"))

        hydro_type = (self.drake_props.get("hydroelastic_type") or "compliant").lower()
        if hydro_type not in ("compliant", "rigid"):
            raise ValueError("drake_props['hydroelastic_type'] must be 'compliant' or 'rigid'")
        ET.SubElement(prox, drake_tag(f"{hydro_type}_hydroelastic"))

        ET.SubElement(prox, drake_tag("hydroelastic_modulus"),
                      value=str(self.drake_props["hydroelastic_modulus"]))
        ET.SubElement(prox, drake_tag("hunt_crossley_dissipation"),
                      value=str(self.drake_props["hunt_crossley_dissipation"]))
        ET.SubElement(prox, drake_tag("resolution_hint"),
                      value=str(self.drake_props["resolution_hint"]))
        ET.SubElement(prox, drake_tag("mu_static"),
                      value=str(self.drake_props["mu_static"]))
        ET.SubElement(prox, drake_tag("mu_dynamic"),
                      value=str(self.drake_props["mu_dynamic"]))

    # -------- NEW: create ONE link and then add many geometries to it --------
    def add_link(self, name):
        return ET.SubElement(self.robot, "link", name=name)

    def add_box_to_link(self, link, geom_name, size, xyz, color):
        """
        Adds a visual + collision (with Drake proximity props) to an EXISTING link.
        geom_name is only used for unique material names (helps debugging).
        """
        # Visual
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "box", size=f"{size[0]} {size[1]} {size[2]}")
        material = ET.SubElement(visual, "material", name=f"{geom_name}_mat")
        ET.SubElement(material, "color", rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}")

        # Collision
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=f"{size[0]} {size[1]} {size[2]}")

        # Drake properties on every collision geometry
        self._add_drake_proximity_properties(collision)

    def set_link_inertial_box_approx(self, link, mass, size, xyz=(0, 0, 0)):
        """
        Single inertial for the whole link.
        We approximate inertia as if it's just the base box (or you can improve this).
        """
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=f"{xyz[0]} {xyz[1]} {xyz[2]}", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(mass))

        x, y, z = size
        ixx = (1/12) * mass * (y**2 + z**2)
        iyy = (1/12) * mass * (x**2 + z**2)
        izz = (1/12) * mass * (x**2 + y**2)

        ET.SubElement(inertial, "inertia",
                      ixx=str(ixx), ixy="0", ixz="0",
                      iyy=str(iyy), iyz="0",
                      izz=str(izz))

    def save(self, filename):
        prettify(self.robot)
        tree = ET.ElementTree(self.robot)
        tree.write(filename, encoding="utf-8", xml_declaration=True)


def generate_robot(
    base_size,
    base_color,
    top_color,
    n,
    top_box_x,
    top_box_z,
    filename,
    drake_props=None
):
    lr, lg, lb, la = top_color
    Lr, Lg, Lb, La = base_color
    Lx, Ly, Lz = base_size

    builder = URDFBuilder(drake_props=drake_props)

    # ✅ ONE link called "terrain"
    terrain = builder.add_link("terrain")

    # Base geometry inside terrain link (centered at origin)
    builder.add_box_to_link(
        link=terrain,
        geom_name="base",
        size=(Lx, Ly, Lz),
        xyz=(0, 0, 0),
        color=(Lr, Lg, Lb, La),
    )

    # Evenly spaced x positions for bumps
    x_min = -Lx/2 + top_box_x/2
    x_max = Lx/2 - top_box_x/2
    if n == 1:
        x_positions = [0.0]
    else:
        x_positions = np.linspace(x_min, x_max, n)

    # Height placement (on top surface)
    z_center = Lz/2 + top_box_z/2

    for i, x in enumerate(x_positions):
        builder.add_box_to_link(
            link=terrain,
            geom_name=f"bump_{i}",
            size=(top_box_x, Ly, top_box_z),
            xyz=(float(x), 0.0, z_center),
            color=(lr, lg, lb, la),
        )

    # Inertial: ONE per link. Approximate using base box.
    # (If this is "ground"/static in Drake, inertia is mostly irrelevant.)
    builder.set_link_inertial_box_approx(
        link=terrain,
        mass=1.0,
        size=(Lx, Ly, Lz),
        xyz=(0, 0, 0),
    )

    builder.save(filename)


if __name__ == "__main__":
    drake_props = {
        "hydroelastic_type": "compliant",   # or "rigid"
        "hydroelastic_modulus": "5e7",      # Pa
        "hunt_crossley_dissipation": "1.0", # s/m
        "resolution_hint": "0.01",          # m
        "mu_static": "0.15",
        "mu_dynamic": "0.12",
    }

    generate_robot(
        base_size=(1.5, 0.5, 0.05),
        base_color=(0.5, 0.5, 0.5, 1.0),
        top_color=(0.4, 0.4, 0.7, 1.0),
        n=75,
        top_box_x=0.01,
        top_box_z=0.005,
        filename="terr_geom.urdf",
        drake_props=drake_props
    )
