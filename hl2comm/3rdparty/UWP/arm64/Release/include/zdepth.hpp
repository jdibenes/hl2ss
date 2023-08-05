// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    Zdepth

    Lossless depth buffer compression designed and tested for Azure Kinect DK.
    Based on the Facebook Zstd library for compression.

    The compressor defines a file format and performs full input checking.
    Supports temporal back-references similar to other video formats.

    We do not use H.264 or other video codecs because the main goal is to
    measure the limits of lossless real-time compression of depth data.
    Another goal is to be 2x smaller than the best published software (RVL).

    Algorithm:

    (1) Quantize depth based on sensor accuracy at range.
    (2) Compress run-lengths of zeroes with Zstd.
    (3) For each 8x8 block of the depth image, determine the best predictor.
    (4) Zig-zag encode and compress the residuals with Zstd.

    The predictors are similar to the way the PNG format works.
*/

#pragma once

#include <stdint.h>
#include <vector>

// Compiler-specific force inline keyword
#if defined(_MSC_VER)
    #define DEPTH_INLINE inline __forceinline
#else // _MSC_VER
    #define DEPTH_INLINE inline __attribute__((always_inline))
#endif // _MSC_VER

// Architecture check
#if defined(__arm__) || defined(_M_ARM)
    // Use aligned accesses on ARM
    #define DEPTH_ALIGNED_ACCESSES
#endif // ANDROID



#ifdef _WIN32
#define ZDEPTH_VISIBILITY_EXPORT __declspec(dllexport)
#else
#define ZDEPTH_VISIBILITY_EXPORT __attribute__((visibility("default")))
#endif

#ifdef _WIN32
#define ZDEPTH_VISIBILITY_INLINE_MEMBER_EXPORT
#else
#define ZDEPTH_VISIBILITY_INLINE_MEMBER_EXPORT __attribute__((visibility("default")))
#endif

#ifdef _WIN32
#define ZDEPTH_VISIBILITY_IMPORT __declspec(dllimport)
#else
#define ZDEPTH_VISIBILITY_IMPORT __attribute__((visibility("default")))
#endif

#define ZDEPTH_VISIBILITY_STATIC

#ifdef _WIN32
#define ZDEPTH_VISIBILITY_LOCAL
#else
#define ZDEPTH_VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#endif


#ifndef ZDEPTH_BUILD_STATIC
#if defined(ZDEPTH_EXPORTS) || defined(zdepth_EXPORTS)
#define ZDEPTH_EXPORT ZDEPTH_VISIBILITY_EXPORT
#else
#define ZDEPTH_EXPORT ZDEPTH_VISIBILITY_IMPORT
#endif
#else
#define ZDEPTH_EXPORT ZDEPTH_VISIBILITY_STATIC
#endif
#define ZDEPTH_LOCAL ZDEPTH_VISIBILITY_LOCAL


namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// First byte of the file format
static const uint8_t kDepthFormatMagic = 202; // 0xCA

// Number of bytes in header
static const int kDepthHeaderBytes = 40;

/*
    File format:

    Format Magic is used to quickly check that the file is of this format.
    Words are stored in little-endian byte order.

    0: <Format Magic = 202 (1 byte)>
    1: <Flags (1 byte)>
    2: <Frame Number (2 bytes)>
    4: <Width (2 bytes)>
    6: <Height (2 bytes)>
    8: <Zeroes Uncompressed Bytes (4 bytes)>
    12: <Zeroes Compressed Bytes (4 bytes)>
    16: <Blocks Uncompressed Bytes (4 bytes)>
    20: <Blocks Compressed Bytes (4 bytes)>
    24: <Edges Uncompressed Bytes (4 bytes)>
    28: <Edges Compressed Bytes (4 bytes)>
    32: <Surfaces Uncompressed Bytes (4 bytes)>
    36: <Surfaces Compressed Bytes (4 bytes)>

    Followed by compressed Zeroes, then Blocks, then Edges, then Surfaces.

    The compressed and uncompressed sizes are of packed data for Zstd.

    Flags = 1 for I-frames and 0 for P-frames.
    The P-frames are able to use predictors that reference the previous frame.
    The decoder keeps track of the previously decoded Frame Number and rejects
    frames that cannot be decoded due to a missing previous frame.
*/

// Size of a block for predictor selection purposes
static const int kBlockSize = 8;

enum class DepthResult
{
    FileTruncated,
    WrongFormat,
    Corrupted,
    MissingPFrame, // Missing previous referenced frame
    BadDimensions, // Width % kBlockSize != 0 and/or Height % kBlockSize != 0
    Success
};

ZDEPTH_EXPORT const char* DepthResultString(DepthResult result);


//------------------------------------------------------------------------------
// Tools

// Little-endian 16-bit read
DEPTH_INLINE uint16_t ReadU16_LE(const void* data)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    const uint8_t* u8p = reinterpret_cast<const uint8_t*>(data);
    return ((uint16_t)u8p[1] << 8) | u8p[0];
#else
    const uint16_t* word_ptr = reinterpret_cast<const uint16_t*>(data);
    return *word_ptr;
#endif
}

// Little-endian 32-bit read
DEPTH_INLINE uint32_t ReadU32_LE(const void* data)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    const uint8_t* u8p = reinterpret_cast<const uint8_t*>(data);
    return ((uint32_t)u8p[3] << 24) | ((uint32_t)u8p[2] << 16) | ((uint32_t)u8p[1] << 8) | u8p[0];
#else
    const uint32_t* u32p = reinterpret_cast<const uint32_t*>(data);
    return *u32p;
#endif
}

// Little-endian 16-bit write
DEPTH_INLINE void WriteU16_LE(void* data, uint16_t value)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    uint8_t* u8p = reinterpret_cast<uint8_t*>(data);
    u8p[1] = static_cast<uint8_t>(value >> 8);
    u8p[0] = static_cast<uint8_t>(value);
#else
    uint16_t* word_ptr = reinterpret_cast<uint16_t*>(data);
    *word_ptr = value;
#endif
}

// Little-endian 32-bit write
DEPTH_INLINE void WriteU32_LE(void* data, uint32_t value)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    uint8_t* u8p = reinterpret_cast<uint8_t*>(data);
    u8p[3] = (uint8_t)(value >> 24);
    u8p[2] = static_cast<uint8_t>(value >> 16);
    u8p[1] = static_cast<uint8_t>(value >> 8);
    u8p[0] = static_cast<uint8_t>(value);
#else
    uint32_t* word_ptr = reinterpret_cast<uint32_t*>(data);
    *word_ptr = value;
#endif
}


bool ZDEPTH_EXPORT IsDepthFrame(const uint8_t* file_data, unsigned file_bytes);
bool ZDEPTH_EXPORT IsKeyFrame(const uint8_t* file_data, unsigned file_bytes);


//------------------------------------------------------------------------------
// Depth Quantization

/*
    Azure Kinect DK sensor whitepaper:
    https://docs.microsoft.com/en-us/windows/mixed-reality/ISSCC-2018

    Minimum operating range = 200 mm.

    The Kinect has 1-2mm accuracy up to about 4 meters.
    Depth uncertainty < 0.2% of range:

        <750 mm  : 1.5 mm precision (or better)
        <1500 mm : 3 mm precision (or better)
        <3000 mm : 6 mm precision (or better)
        <6000 mm : 12 mm precision (or better)
        <12000 mm : 24 mm precision (or better)

    Our quantization table:

        [0, 200] mm      -> 0            (no depth data)
        [201, 750) mm    -> [1, 550)     (lossless)
        [750, 1500) mm   -> [550, 925)   (quantized 2x)
        [1500, 3000) mm  -> [925, 1300)  (quantized 4x)
        [3000, 6000) mm  -> [1300, 1675) (quantized 8x)
        [6000, 11840) mm -> [1675, 2040) (quantized 16x)
        Larger depth     -> 0            (no depth data)

    Reverse quantization table:

        0            -> 0                (no depth data)
        [1, 550)     -> [201, 750) mm    (lossless)
        [550, 925)   -> [750, 1500) mm   (quantized 2x)
        [925, 1300)  -> [1500, 3000) mm  (quantized 4x)
        [1300, 1675) -> [3000, 6000) mm  (quantized 8x)
        [1675, 2040) -> [6000, 11840) mm (quantized 16x)
        Larger values are invalid.
*/

// Quantize depth from 200..11840 mm to a value from 0..2040
uint16_t ZDEPTH_EXPORT AzureKinectQuantizeDepth(uint16_t depth);

// Reverse quantization back to original depth
uint16_t ZDEPTH_EXPORT AzureKinectDequantizeDepth(uint16_t quantized);

// Quantize depth for a whole image
void ZDEPTH_EXPORT QuantizeDepthImage(
    int width,
    int height,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized);
void ZDEPTH_EXPORT DequantizeDepthImage(
    int width,
    int height,
    const uint16_t* quantized,
    std::vector<uint16_t>& depth);


//------------------------------------------------------------------------------
// Zstd

void ZDEPTH_EXPORT ZstdCompress(
    const std::vector<uint8_t>& uncompressed,
    std::vector<uint8_t>& compressed);

bool ZDEPTH_EXPORT ZstdDecompress(
    const uint8_t* compressed_data,
    int compressed_bytes,
    int uncompressed_bytes,
    std::vector<uint8_t>& uncompressed);


//------------------------------------------------------------------------------
// Pack12

/*
    Pack12 Format Description:

    This packing format is optimized for depth data compression.
    Precondition: There are an even number of 12-bit values to write.

    For N inputs the first N output bytes form the `packed0` plane.
    The next N/2 output bytes form the `packed1` plane.

    The `packed0` plane consists of the high 8 bytes of all the inputs.
    The `packed1` plane consists of low bits of pairs of inputs.
*/

// Pad data with a zero entry to make its length even
void ZDEPTH_EXPORT Pad12(std::vector<uint16_t>& data);

// Pack 12-bit fields into bytes for Zstd compression.
// Input must be a multiple of two in size
void ZDEPTH_EXPORT Pack12(
    const std::vector<uint16_t>& data,
    std::vector<uint8_t>& packed);

void ZDEPTH_EXPORT Unpack12(
    const std::vector<uint8_t>& packed,
    std::vector<uint16_t>& data);


//------------------------------------------------------------------------------
// DepthCompressor

class ZDEPTH_EXPORT DepthCompressor
{
public:
    // Compress depth array to buffer
    // Set keyframe to indicate this frame should not reference the previous one
    // Returns DepthResult::Success if the depth can be compressed
    DepthResult Compress(
        int width,
        int height,
        const uint16_t* unquantized_depth,
        std::vector<uint8_t>& compressed,
        bool keyframe);

    // Decompress buffer to depth array.
    // Resulting depth buffer is row-first, stride=width*2 (no surprises).
    // Returns DepthResult::Success if original depth can be recovered
    DepthResult Decompress(
        const std::vector<uint8_t>& compressed,
        int& width,
        int& height,
        std::vector<uint16_t>& depth_out);

protected:
    // Depth values quantized for current and last frame
    std::vector<uint16_t> QuantizedDepth[2];
    unsigned CurrentFrameIndex = 0;
    unsigned CompressedFrameNumber = 0;

    // Accumulated through the end of the filtering then compressed separately
    std::vector<uint16_t> Edges, Surfaces;

    // Block descriptors
    std::vector<uint8_t> Zeroes, Blocks;

    int Zeroes_UncompressedBytes = 0;
    int Surfaces_UncompressedBytes = 0;
    int Blocks_UncompressedBytes = 0;
    int Edges_UncompressedBytes = 0;

    // Results of zstd compression
    std::vector<uint8_t> ZeroesOut, SurfacesOut, BlocksOut, EdgesOut;

    // Packs the 16-bit overruns into 12-bit values and apply Zstd
    std::vector<uint8_t> Packed;


    void CompressImage(
        int width,
        int height,
        const uint16_t* depth,
        const uint16_t* prev_depth);
    bool DecompressImage(
        int width,
        int height,
        uint16_t* depth,
        const uint16_t* prev_depth);

    void WriteCompressedFile(
        int width,
        int height,
        bool keyframe,
        std::vector<uint8_t>& compressed);

    void EncodeZeroes(
        int width,
        int height,
        const uint16_t* depth);

    // This writes the whole QuantizedDepth image with 0 or 1
    // before the block decoding starts.
    void DecodeZeroes(
        int width,
        int height,
        uint16_t* depth);
};


} // namespace zdepth
