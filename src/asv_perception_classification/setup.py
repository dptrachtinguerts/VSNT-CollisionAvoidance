from setuptools import find_packages, setup

package_name = 'asv_perception_classification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vicente',
    maintainer_email='vicente@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "video_publisher = asv_perception_classification.video_publisher:main",
            "yolo_subscriber = asv_perception_classification.yolo_subscriber:main"
        ],
    },
)
