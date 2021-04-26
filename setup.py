
from setuptools import setup, find_packages


setup(name='beeclient-drop-scanner',
      description='beeBox Drop Scanner',
      packages=find_packages(),
      install_requires=[],
      zip_safe=False,
      use_scm_version={"root": ".", "relative_to": __file__},
      setup_requires=['setuptools-scm', 'wheel'],
      namespace_packages=['beeclient']
      )
