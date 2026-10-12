from glob import glob

from setuptools import find_packages, setup

package_name = 'QuadPkg'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'legacy', 'tools', 'isaac']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # robot model (joint 0 = standing) + its meshes  ->  share/QuadPkg/urdf/
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf/meshes', glob('urdf/meshes/*')),
        # start-up recipes and parameter files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # documentation (handy on the Jetson: ~/quadruped_ws/install/QuadPkg/share/QuadPkg/docs)
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikunj',
    maintainer_email='183414930+iota-nik@users.noreply.github.com',
    description='Quadruped trot controller for Isaac Sim and the real robot (Jetson + ESP32 servo relay)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ---- run the robot ----
            'controller = QuadPkg.gait.gait:main',            # gait brain (sim AND real)
            'gait = QuadPkg.gait.gait:main',                  # old name, same node
            'teleop = QuadPkg.teleop.keyboard_teleop:main',   # keyboard driving
            'hardware_bridge = QuadPkg.hardware.bridge:main',  # Jetson <-> ESP32 <-> servos
            # ---- setup ----
            'servo_tool = QuadPkg.hardware.servo_tool:main',  # scan|read|limp|hold|stand|jog|calib|middle|setid
            'export_urdf = QuadPkg.description.export_urdf:main',
            # ---- debugging ----
            'diagnose = QuadPkg.debug.diagnose:main',          # publishers, clock, tracking error
            'servo_monitor = QuadPkg.debug.servo_monitor:main',  # live table: sent vs feedback per servo
            'check_robot = QuadPkg.debug.check_robot:main',    # mass / COM / torque / URDF checks (no ROS)
        ],
    },
)
