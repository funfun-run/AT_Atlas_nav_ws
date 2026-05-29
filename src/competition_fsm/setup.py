from setuptools import setup

package_name = 'competition_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='funfun',
    maintainer_email='1219921425@qq.com',
    description='Competition FSM node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'competition_fsm = competition_fsm.fsm_node:main',
        ],
    },
)
