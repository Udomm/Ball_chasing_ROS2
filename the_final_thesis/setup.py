from setuptools import setup

package_name = 'the_final_thesis'

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
    maintainer='udom',
    maintainer_email='udom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'color_detection = the_final_thesis.color_detection:main',
        'camera_publisher = the_final_thesis.camera_publisher:main',
        'webcam_sub = the_final_thesis.webcam_sub:main',
        'webcam_pub = the_final_thesis.webcam_pub:main',
        'ball_detection = the_final_thesis.ball_detection:main',
        'ball_chasing = the_final_thesis.ball_chasing:main'
        
        ],
    },
)
