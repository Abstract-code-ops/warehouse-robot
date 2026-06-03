import os
from glob import glob
from setuptools import setup

package_name = 'cognitive_amr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Install helper scripts so colcon includes them
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    zip_safe=True,
    maintainer='sicheen',
    maintainer_email='aa2933395@gmail.com',
    description='Cognitive AMR Semantic Planner',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'inventory_manager   = cognitive_amr.inventory_manager_node:main',
            'aruco_detector      = cognitive_amr.aruco_detector_node:main',
            'tag_localization    = cognitive_amr.tag_localization_node:main',
            'task_planner        = cognitive_amr.task_planner_node:main',
            'operator_interface  = cognitive_amr.operator_interface_node:main',
            'pick_simulator      = cognitive_amr.pick_simulator_node:main',
            'llm_gateway         = cognitive_amr.llm_gateway_node:main',
            'operator_web_ui     = cognitive_amr.operator_web_ui:main',
        ],
    },
)
