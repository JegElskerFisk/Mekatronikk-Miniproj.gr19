from setuptools import setup

package_name = 'qube_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/qube.urdf.xacro', 'urdf/qube.macro.xacro']),
        ('share/' + package_name + '/rviz', ['rviz/view_qube.rviz']),
        ('share/' + package_name + '/launch', ['launch/view_qube.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='per',
    maintainer_email='your@email.com',
    description='URDF description of the Qube robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)

