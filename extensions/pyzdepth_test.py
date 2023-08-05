
import pyzdepth
import numpy as np

data = np.zeros((512, 512), dtype=np.uint16)
t = data.tobytes()

compressor = pyzdepth.DepthCompressor()
result, compressed = compressor.Compress(512, 512, t, 1)
print(result)
print(len(compressed))

decompressor = pyzdepth.DepthCompressor()
result, width, height, decompressed = decompressor.Decompress(compressed)
print(result)
print(width)
print(height)
print(len(decompressed))
