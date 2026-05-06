from setuptools import find_packages, setup
import os

package_name = 'ur10_pbvs_sim'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        #Simulation Directories
        (os.path.join('share', package_name, 'launch'), package_files('launch')),
        (os.path.join('share', package_name, 'urdf'), package_files('urdf')),
        (os.path.join('share', package_name, 'config'), package_files('config')),
        (os.path.join('share', package_name, 'meshes'), package_files('meshes')),
        (os.path.join('share', package_name, 'worlds'), package_files('worlds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gonzcode',
    maintainer_email='gonzi.ceron.denetro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
