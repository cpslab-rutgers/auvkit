from setuptools import setup

setup(name='AUVKit',
      version='0.1',
      description='A Python library for communicating, controlling and writing apps for autonomous underwater vehicles.',
      author='Mohammad Nadeem',
      author_email='mohammednadeem902@gmail.com',
      packages=['auvkit'],
      install_requires=['pymavlink', 'bluerobotics-ping']
      )
