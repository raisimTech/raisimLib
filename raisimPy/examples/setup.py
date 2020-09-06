# Once in the examples folder, type one of the following command:
# $ pip install -e .
# $ pip install --user -e .
# $ python setup.py install
# $ python setup.py install --home=<dir>

from setuptools import setup, find_packages
try: # for pip >= 10
    from pip._internal.req import parse_requirements
except ImportError: # for pip <= 9.0.3
    from pip.req import parse_requirements


# get the required packages
install_requires = parse_requirements('requirements.txt', session=False)
reqs = [str(ir.req) for ir in install_requires]

# setup
setup(
    name='raisimpy_gym',
    version='0.1.0',
    description='RaiSimPy Gym',
    author=['Jemin Hwangbo (raisim_gym)', 'Brian Delhaisse (raisimpy + raisimpy_gym)'],
    maintainer='Brian Delhaisse',
    maintainer_email='briandelhaisse@gmail.com',
    license='MIT',
    url='https://github.com/robotlearn/raisimpy',
    platforms=['Linux Ubuntu'],
    packages=find_packages(),  # find_packages(exclude=('tests',))
    install_requires=reqs,
    classifiers=[
        "Programming Language :: Python :: 3.5",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
    ],
)
