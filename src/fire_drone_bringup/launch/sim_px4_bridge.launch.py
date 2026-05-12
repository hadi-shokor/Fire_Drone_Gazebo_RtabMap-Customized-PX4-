import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    px4_dir = os.path.expanduser("~/PX4-Autopilot")

    
    px4_sitl = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"cd {px4_dir} && PX4_GZ_WORLD=baylands make px4_sitl gz_x500"
        ],
        output="screen"
    )

    
    xrce_agent = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        px4_sitl,
        xrce_agent
    ])
