from setuptools import setup

package_name = 'qos_tests_pkg'

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
    maintainer='kevin',
    maintainer_email='907575489@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'publisher_custom_minimal_qos_exe = qos_tests_pkg.publisher_custom_minimal_qos:main',
        	'subscriber_custom_minimal_qos_exe = qos_tests_pkg.subscriber_custom_minimal_qos:main',  
        ],
    },
)
