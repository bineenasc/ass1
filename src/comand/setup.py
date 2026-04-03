from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'comand'



def package_files(data_files, directory, install_dir):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)

            install_path = os.path.join(
                install_dir,
                os.path.relpath(path, directory)
            )

            paths.append((install_path, [file_path]))

    data_files.extend(paths)
    return data_files



data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),

    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.sdf')),
]


data_files = package_files(
    data_files,
    'models',
    os.path.join('share', package_name, 'models')
)



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bob5t0nl0v3r',
    maintainer_email='bob5t0nl0v3r@todo.todo',
    description='Gazebo simulation launcher',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'spawn_robot = comand.spawn_robot_node:main',
        ],
    }
)