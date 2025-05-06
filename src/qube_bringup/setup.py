from setuptools import find_packages, setup

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
      ('share/ament_index/resource_index/packages',['resource/'+package_name]),
      ('share/'+package_name,['package.xml']),
      ('share/'+package_name+'/urdf', ['urdf/controlled_qube.urdf.xacro']),
      ('share/'+package_name+'/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='per',
    maintainer_email='per@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
