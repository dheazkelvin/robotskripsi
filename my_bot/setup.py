from setuptools import setup

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer_name',
    maintainer_email='maintainer_email',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_pose = my_bot.go_to_pose:main',
            'go_to_pose2 = my_bot.go_to_pose2:main',
            'go_to_pose3 = my_bot.go_to_pose3:main',
            'go_to_pose4 = my_bot.go_to_pose4:main',
            'go_to_pose5 = my_bot.go_to_pose5:main',
            'qos_bridge = my_bot.qos_bridge:main',
            'sound_player = my_bot.sound_tester:main',
        ],
    },
)

