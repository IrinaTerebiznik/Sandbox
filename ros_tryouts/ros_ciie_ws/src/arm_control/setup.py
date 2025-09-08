from setuptools import setup

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Simple serial bridge to Arduino for preset arm commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_serial_bridge = arm_control.arm_serial_bridge:main',
        ],
    },
)
