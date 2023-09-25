
from distutils.core import setup
from distutils.extension import Extension
from glob import glob

path_Zdepth = '../../3rdparty/Zdepth'
path_zstd   = '../../3rdparty/Zdepth/zstd'

Zdepth_cpp = [x for x in glob(f'{path_Zdepth}/src/*.cpp')]
Zdepth_c   = [x for x in glob(f'{path_Zdepth}/src/*.c')]
zstd_cpp   = [x for x in glob(f'{path_zstd}/src/*.cpp')]
zstd_c     = [x for x in glob(f'{path_zstd}/src/*.c')]

sources = Zdepth_cpp + Zdepth_c + zstd_cpp + zstd_c + ['./pyzdepth.cpp']

ext = Extension('pyzdepth',
                sources = sources,
                language = 'c++',
                include_dirs=[path_Zdepth + '/include', path_zstd + '/include', path_zstd + '/src'])

setup(name='pyzdepth', ext_modules=[ext])
