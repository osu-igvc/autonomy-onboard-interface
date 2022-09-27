from setuptools import setup

package_name = 'autonomy_hmi'
ui_name = 'window.ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + ui_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'style.qss']),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igvcsp2022',
    maintainer_email='max.desantis@okstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'primary_interface = autonomy_hmi.primary_interface:main',
            'gui_node = autonomy_hmi.gui_node:main'
        ],
    },
)
