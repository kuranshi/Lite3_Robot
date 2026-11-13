from setuptools import find_packages, setup

setup(name='lite3_recovery_telemetry', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/lite3_recovery_telemetry']),
                  ('share/lite3_recovery_telemetry', ['package.xml', 'LICENSE-MIT'])],
      install_requires=['setuptools'], zip_safe=True, license='Apache-2.0 AND MIT',
      maintainer='Lite3 Contributors', maintainer_email='contributors@example.invalid',
      entry_points={'console_scripts': ['native_state = lite3_recovery_telemetry.receiver:main']})
