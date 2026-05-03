from setuptools import setup

setup(
    name='cognitive_amr_gazebo',
    version='0.1.0',
    packages=['cognitive_amr_gazebo'],
    entry_points={
        'console_scripts': [
            'laser_merger = cognitive_amr_gazebo.laser_merger:main',
            'task_dispatcher = cognitive_amr_gazebo.task_dispatcher:main',
        ],
    },
)
