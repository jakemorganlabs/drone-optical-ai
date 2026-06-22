from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import pybind11
import sys
import os

# Custom build_ext to add compiler-specific flags
class BuildExt(build_ext):
    c_opts = {
        'msvc': ['/O2', '/openmp'],  # Enable optimization and OpenMP for MSVC
        'unix': ['-O3', '-fopenmp'],
    }
    l_opts = {
        'msvc': [],
        'unix': ['-fopenmp'],
    }

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        link_opts = self.l_opts.get(ct, [])
        for ext in self.extensions:
            ext.extra_compile_args = opts
            ext.extra_link_args = link_opts
        build_ext.build_extensions(self)

# Desktop target (for development and visualization)
desktop_extensions = [
    Extension(
        'process_image_cpp',
        ['process_image.cpp'],
        include_dirs=[
            pybind11.get_include(),
        ],
        language='c++'
    ),
]

# Embedded target (for live camera operation)
embedded_extensions = [
    Extension(
        'live_voxel_mapper',
        ['live_voxel_mapper.cpp'],
        include_dirs=[
            pybind11.get_include(),
        ],
        language='c++'
    ),
]

# Choose extensions based on environment variable
ext_modules = desktop_extensions
if os.environ.get('BUILD_EMBEDDED', '').lower() in ('1', 'true', 'yes'):
    ext_modules = embedded_extensions
    print("Building embedded target for live camera operation")
else:
    print("Building desktop target for development and visualization")

setup(
    name='voxel_mapping_system',
    version='1.0.0',
    description='Live camera-based voxel mapping with motion detection and navigation',
    ext_modules=ext_modules,
    cmdclass={'build_ext': BuildExt},
    python_requires='>=3.7',
    install_requires=[
        'numpy>=1.19.0',
        'pybind11>=2.6.0',
    ],
    extras_require={
        'desktop': [
            'matplotlib>=3.3.0',
            'astropy>=4.0.0',
        ],
        'embedded': [
            'opencv-python>=4.5.0',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: C++',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Computer Vision',
    ],
)
