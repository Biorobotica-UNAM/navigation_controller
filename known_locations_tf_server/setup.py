from setuptools import setup

package_name = 'known_locations_tf_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Angel',
    author_email='angel@example.com',
    description='Known locations TF server',
    entry_points={
        'console_scripts': [
            'known_locs_node = known_locations_tf_server.known_locs_node:main',
            'known_locs_node_nav = known_locations_tf_server.known_locs_node_nav:main',
        ],
    },
)




