from setuptools import setup

setup(
   name='skateboard_gui',
   version='1.0',
   description='A useful module',
   author='Sujith Christopher',
   author_email='chrisbon95@gmail.com',
   packages=['skateboard_gui'],  #same as name
   install_requires=['matplotlib', 'opencv-contrib-python', 'pandas', 'numpy', 'fpdf'], #external packages as dependencies
)