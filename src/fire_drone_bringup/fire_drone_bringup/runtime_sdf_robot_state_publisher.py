import subprocess


def get_model_info(model_name: str) -> str:
    cmd = ["gz", "model", "-m", model_name]
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        check=True
    )
    return result.stdout


def parse_links_only(model_info: str):
    links = []
    lines = model_info.splitlines()
    i = 0

    while i < len(lines):
        line = lines[i]

        if line.startswith("  - Link ["):
            link_name = None
            xyz = None
            rpy = None

            i += 1
            while i < len(lines):
                current = lines[i]

                if current.startswith("  - Link [") or current.startswith("  - Joint ["):
                    i -= 1
                    break

                if current.startswith("    - Name:"):
                    link_name = current.split(":", 1)[1].strip()

                if current.startswith("    - Pose [ XYZ (m) ] [ RPY (rad) ]:"):
                    xyz = lines[i + 1].strip().replace("[", "").replace("]", "")
                    rpy = lines[i + 2].strip().replace("[", "").replace("]", "")

                i += 1

            if link_name and xyz and rpy:
                links.append((link_name, xyz, rpy))

        i += 1

    return links


def visual_for_link(link_name: str) -> str:
    if link_name == "base_link":
        return """
    <visual>
      <geometry>
        <mesh filename="package://fire_drone_bringup/meshes/NXP-HGD-CF.dae"/>
      </geometry>
    </visual>
"""

    if link_name in ["rotor_0", "rotor_1"]:
        return """
    <visual>
      <geometry>
        <mesh filename="package://fire_drone_bringup/meshes/1345_prop_ccw.stl"/>
      </geometry>
    </visual>
"""

    if link_name in ["rotor_2", "rotor_3"]:
        return """
    <visual>
      <geometry>
        <mesh filename="package://fire_drone_bringup/meshes/1345_prop_cw.stl"/>
      </geometry>
    </visual>
"""

    return ""


def generate_urdf(links):
    urdf = []
    used_links = set()
    used_joints = set()

    urdf.append('<?xml version="1.0"?>')
    urdf.append('<robot name="fire_drone">')

    urdf.append(f'''
  <link name="base_link">
    {visual_for_link("base_link")}
  </link>
''')

    used_links.add("base_link")

    for link_name, xyz, rpy in links:
        if link_name == "base_link":
            continue

        if link_name in used_links:
            continue

        used_links.add(link_name)

        joint_name = f"base_to_{link_name}"

        if joint_name in used_joints:
            continue

        used_joints.add(joint_name)

        urdf.append(f'''
  <link name="{link_name}">
    {visual_for_link(link_name)}
  </link>

  <joint name="{joint_name}" type="fixed">
    <parent link="base_link"/>
    <child link="{link_name}"/>
    <origin xyz="{xyz}" rpy="{rpy}"/>
  </joint>
''')

    urdf.append('</robot>')
    return "\n".join(urdf)


def main():
    model_name = "x500_fire_drone_0"

    model_info = get_model_info(model_name)
    links = parse_links_only(model_info)
    urdf = generate_urdf(links)

    output_path = "/tmp/fire_drone_runtime.urdf"

    with open(output_path, "w") as f:
        f.write(urdf)

    print(f"\nGenerated runtime URDF:\n{output_path}\n")
    print("Links added:")
    for link_name, _, _ in links:
        print(f"  - {link_name}")


if __name__ == "__main__":
    main()
