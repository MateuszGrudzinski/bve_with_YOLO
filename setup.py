from setuptools import setup

package_name = 'bird_view'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Camera viewer node using OpenCV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_subscriber = bird_view.camera_subscriber:main',
        'lidar_subscriber = bird_view.lidar_subscriber:main',
        'lidar_bird_view = bird_view.bird_eye_view:main',
        'autopilot_display = bird_view.autopilot_display:main',
        'camera_lidar_fusion = bird_view.camera_inference:main',
        'testTF = bird_view.testTF:main',
    ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/bird_view']),
        ('share/bird_view', ['package.xml']),
        ('share/bird_view/launch', ['launch/bve_launch.py']),
        ('share/bird_view/launch', ['launch/bve_autopilot_launch.py']),
        ('share/bird_view/launch', ['launch/autoware_min_launch.py']),
    ],
    package_data={
            # This assumes your resources are inside the bird_view folder or a subfolder of it
            package_name: ['resource/models/*.obj'],
        },
)
