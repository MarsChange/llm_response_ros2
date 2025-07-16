from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'llm_response_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools','gradio_client'],
    zip_safe=True,
    maintainer='martin chen',
    maintainer_email='marccc@qq.com',
    description='ROS2 package for LLM response via API',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_response_node = llm_response_package.llm_response_node:main',
        ],
    },
) 