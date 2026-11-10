from setuptools import setup
package_name='rover_bringup'
setup(name=package_name,version='0.1.0',packages=[package_name],data_files=[
 ('share/ament_index/resource_index/packages',['resource/'+package_name]),
 ('share/'+package_name,['package.xml']),
 ('share/'+package_name+'/launch',['launch/rover.launch.py','launch/sim.launch.py']),
 ('share/'+package_name+'/config',['config/rover_config.yaml','config/nav2_params.yaml']),
],install_requires=['setuptools','pyserial'],zip_safe=True,maintainer='Rover Builder',maintainer_email='rover@example.com',description='ROS 2 bringup and exploration controls for a mapping rover',license='Apache-2.0',entry_points={'console_scripts':['base_bridge = rover_bringup.base_bridge:main','explore_keys = rover_bringup.explore_keys:main','safety_gate = rover_bringup.safety_gate:main','frontier_explorer = rover_bringup.frontier_explorer:main']})
