from distutils.core import setup

install_requires = [
    'numpy',
    'urdf_parser_py'
    ]

test_requires = [
    'modern_robotics',
    'nose'
    ]

setup(
    name='urdf_extract_screw_axes',
    version='0.1dev',
    author=['Jarvis Schultz'],
    author_email='schultzjarvis@gmail.com',
    packages=['urdf_extract_screw_axes',],
    description='Tool for converting URDF robot descriptions to screw axes',
    long_description=open('README.org').read(),
    install_requires=install_requires,
    test_requires=test_requires
)
