from setuptools import setup

install_requires = [
    'numpy',
    'urdf_parser_py',
    # 'kdl_parser_py'
    ]

tests_require = [
    'modern_robotics',
    'nose'
    ]

setup(
    name='urdf_extract_screw_axes',
    version='0.1.dev0',
    author=['Jarvis Schultz'],
    author_email='schultzjarvis@gmail.com',
    packages=['urdf_extract_screw_axes',],
    description='Tool for converting URDF robot descriptions to screw axes',
    long_description=open('README.org').read(),
    install_requires=install_requires,
    tests_require=tests_require,
    test_suite='nose.collector'
)
