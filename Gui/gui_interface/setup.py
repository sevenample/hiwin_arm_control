from setuptools import setup

package_name = 'gui_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='shaoan',
    maintainer_email='shaoan@todo.todo',
    description='PyQt5 GUI interface for order input',
    license='MIT-0',
    entry_points={
        'console_scripts': [
            'hshs_gui = gui_interface.hshs_gui:main',
            'order_listener = gui_interface.order_listener:main'
        ],
    },
)
