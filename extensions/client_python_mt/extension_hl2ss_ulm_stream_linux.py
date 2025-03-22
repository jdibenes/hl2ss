
from distutils.core import setup
from distutils.extension import Extension

sources = ['./hl2ss_ulm_stream.cpp']

ext = Extension('hl2ss_ulm_stream',
                sources = sources,
                language = 'c++',
                include_dirs=['../client_cpp/'],
                library_dirs=['../client_cpp/'],
                libraries=['hl2ss_ulm'],
                extra_link_args=["-Wl,-rpath,$ORIGIN"],
                )

setup(name='hl2ss_ulm_stream', ext_modules=[ext])
