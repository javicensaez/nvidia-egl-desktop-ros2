import os
from glob import glob
from setuptools import find_packages, setup
from itertools import chain

package_name = 'socialtech_challenge'

def generate_data_files(dirs=['launch', 'worlds', 'models','urdf','config','meshes']):
    """
    Generate recursive list of data files, without listing directories in the output.
    """
    data_files = []
    for path, _, files in chain.from_iterable(os.walk(dir) for dir in dirs):
        install_dir = path[len('gazebo/'):] if path.startswith('gazebo/') else path  # remove gazebo/ prefix
        install_dir = os.path.join('share', package_name, install_dir)
        list_entry = (install_dir, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+ generate_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javivi',
    maintainer_email='javivi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
