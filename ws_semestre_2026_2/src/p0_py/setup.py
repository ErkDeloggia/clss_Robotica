from setuptools import find_packages, setup

package_name = 'p0_py'

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
    maintainer='arrgusr',
    maintainer_email='erik.pena@ingenieria.unam.edu',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'primer_node_py = p0_py.primer_nodo:main',
            'subscriber_node_py = p0_py.subscriber_node:main',
            'nodo_pub_sndl = p0_py.nodo_pub_sndl:main',
            'nodo_sub_pub_rads = p0_py.nodo_sub_pub_rads:main'
        ],
    },
)
