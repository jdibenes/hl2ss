// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"
#include <zstd.h> // Zstd
#include <string.h> // memcpy

namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// Zstd compression level
static const int kZstdLevel = 1;

const char* DepthResultString(DepthResult result)
{
    switch (result)
    {
    case DepthResult::FileTruncated: return "FileTruncated";
    case DepthResult::WrongFormat: return "WrongFormat";
    case DepthResult::Corrupted: return "Corrupted";
    case DepthResult::MissingPFrame: return "MissingPFrame";
    case DepthResult::BadDimensions: return "BadDimensions";
    case DepthResult::Success: return "Success";
    default: break;
    }
    return "Unknown";
}


//------------------------------------------------------------------------------
// Tools

bool IsDepthFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (file_bytes < kDepthHeaderBytes) {
        return false;
    }
    if (file_data[0] != kDepthFormatMagic) {
        return false;
    }
    return true;
}

bool IsKeyFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (!IsDepthFrame(file_data, file_bytes)) {
        return false;
    }
    return (file_data[1] & 1) != 0;
}


//------------------------------------------------------------------------------
// Depth Quantization

uint16_t AzureKinectQuantizeDepth(uint16_t depth)
{
    if (depth <= 200) {
        return 0; // Too close
    }
    if (depth < 750) {
        return depth - 200;
    }
    if (depth < 1500) {
        return 550 + (depth - 750) / 2;
    }
    if (depth < 3000) {
        return 925 + (depth - 1500) / 4;
    }
    if (depth < 6000) {
        return 1300 + (depth - 3000) / 8;
    }
    if (depth < 11840) {
        return 1675 + (depth - 6000) / 16;
    }
    return 0; // Too far
}

uint16_t AzureKinectDequantizeDepth(uint16_t quantized)
{
    if (quantized == 0) {
        return 0;
    }
    if (quantized < 550) {
        return quantized + 200;
    }
    if (quantized < 925) {
        return 750 + (quantized - 550) * 2;
    }
    if (quantized < 1300) {
        return 1500 + (quantized - 925) * 4;
    }
    if (quantized < 1675) {
        return 3000 + (quantized - 1300) * 8;
    }
    if (quantized < 2040) {
        return 6000 + (quantized - 1675) * 16;
    }
    return 0; // Invalid value
}

void QuantizeDepthImage(
    int width,
    int height,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized)
{
    const int n = width * height;
    quantized.resize(n);
    uint16_t* dest = quantized.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectQuantizeDepth(depth[i]);
    }
}

void DequantizeDepthImage(
    int width,
    int height,
    const uint16_t* quantized,
    std::vector<uint16_t>& depth)
{
    const int n = width * height;
    depth.resize(n);
    uint16_t* dest = depth.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectDequantizeDepth(quantized[i]);
    }
}


//------------------------------------------------------------------------------
// Depth Predictors

// Supported predictor functions
enum PredictorTypes
{
    PredictorType_Larger,
    PredictorType_Up,
    PredictorType_Left,
    PredictorType_UpTrend,
    PredictorType_LeftTrend,
    PredictorType_Average,

    // The following predictor types sample the previous frame:
    PredictorType_PrevFrame,

    // Number of predictors implemented
    PredictorType_Count
};

static inline unsigned ApplyPrediction(int depth, int prediction)
{
    // Apply prediction
    const int32_t delta = static_cast<int32_t>( depth - prediction );

    // Zig-zag encoding to make the signed number positive
    return (delta << 1) ^ (delta >> 31);
}

static inline int UndoPrediction(unsigned zigzag, int prediction)
{
    // Undo zig-zag encoding to get signed number
    const int delta = (zigzag >> 1) ^ -static_cast<int32_t>(zigzag & 1);

    // Undo prediction
    return delta + prediction;
}

static inline int Predict_Larger(int left0, int up0)
{
    return left0 > up0 ? left0 : up0;
}

static inline int Predict_Up(int left0, int up0)
{
    return up0 != 0 ? up0 : left0;
}

static inline int Predict_Left(int left0, int up0)
{
    return left0 != 0 ? left0 : up0;
}

static inline int Predict_UpTrend(int left0, int up0, int up1)
{
    if (up0 && up1) {
        return 2 * up0 - up1;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_LeftTrend(int left0, int left1, int up0)
{
    if (left0 && left1) {
        return 2 * left0 - left1;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_Average(int left0, int up0)
{
    if (left0 && up0) {
        return (left0 + up0) / 2;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_PrevFrame(int prev0, int left0, int up0)
{
    if (prev0) {
        return prev0;
    }
    return Predict_Larger(left0, up0);
}


//------------------------------------------------------------------------------
// Zstd

void ZstdCompress(
    const std::vector<uint8_t>& uncompressed,
    std::vector<uint8_t>& compressed)
{
    compressed.resize(ZSTD_compressBound(uncompressed.size()));
    const size_t size = ZSTD_compress(
        compressed.data(),
        compressed.size(),
        uncompressed.data(),
        uncompressed.size(),
        kZstdLevel);
    if (ZSTD_isError(size)) {
        compressed.clear();
        return;
    }
    compressed.resize(size);
}

bool ZstdDecompress(
    const uint8_t* compressed_data,
    int compressed_bytes,
    int uncompressed_bytes,
    std::vector<uint8_t>& uncompressed)
{
    uncompressed.resize(uncompressed_bytes);
    const size_t size = ZSTD_decompress(
        uncompressed.data(),
        uncompressed.size(),
        compressed_data,
        compressed_bytes);
    if (ZSTD_isError(size)) {
        return false;
    }
    if (size != static_cast<size_t>( uncompressed_bytes )) {
        return false;
    }
    return true;
}


//------------------------------------------------------------------------------
// Pack12

void Pad12(std::vector<uint16_t>& data)
{
    if (data.size() % 2 != 0) {
        data.push_back(0);
    }
}

void Pack12(
    const std::vector<uint16_t>& unpacked,
    std::vector<uint8_t>& packed)
{
    unsigned data_count = static_cast<unsigned>( unpacked.size() );
    packed.resize(data_count + (data_count / 2));
    uint8_t* packed0 = packed.data();
    uint8_t* packed1 = packed0 + data_count;
    const uint16_t* data = unpacked.data();
    const uint16_t* data_end = data + data_count;
    while (data < data_end)
    {
        const uint16_t x = data[0];
        const uint16_t y = data[1];
        data += 2;

        packed0[0] = static_cast<uint8_t>( x >> 4 );
        packed0[1] = static_cast<uint8_t>( y >> 4 );
        packed0 += 2;

        packed1[0] = static_cast<uint8_t>( (x & 15) | ((y & 15) << 4) );
        packed1++;
    }
}

void Unpack12(
    const std::vector<uint8_t>& packed,
    std::vector<uint16_t>& unpacked)
{
    unsigned data_count = static_cast<unsigned>(packed.size()) * 2 / 3;
    unpacked.resize(data_count);
    const uint8_t* packed0 = packed.data();
    const uint8_t* packed1 = packed0 + data_count;
    uint16_t* data = unpacked.data();
    const uint16_t* data_end = data + data_count;
    while (data < data_end)
    {
        uint16_t x = static_cast<uint16_t>( packed0[0] ) << 4;
        uint16_t y = static_cast<uint16_t>( packed0[1] ) << 4;
        packed0 += 2;

        const uint8_t p1 = packed1[0];
        x |= p1 & 15;
        y |= p1 >> 4;
        packed1++;

        data[0] = x;
        data[1] = y;
        data += 2;
    }
}


//------------------------------------------------------------------------------
// DepthCompressor

DepthResult DepthCompressor::Compress(
    int width,
    int height,
    const uint16_t* unquantized_depth,
    std::vector<uint8_t>& compressed,
    bool keyframe)
{
    // Enforce that image dimensions are multiples of the block size
    if(width % kBlockSize != 0 || height % kBlockSize != 0) {
        return DepthResult::BadDimensions;
    }

    // Enforce keyframe if we have not compressed anything yet
    if (CompressedFrameNumber == 0) {
        keyframe = true;
    }
    ++CompressedFrameNumber;

    // Quantize the depth image
    QuantizeDepthImage(width, height, unquantized_depth, QuantizedDepth[CurrentFrameIndex]);

    // Get depth for previous frame
    const uint16_t* depth = QuantizedDepth[CurrentFrameIndex].data();

    CurrentFrameIndex = (CurrentFrameIndex + 1) % 2;
    const uint16_t* prev_depth = nullptr;
    if (!keyframe) {
        prev_depth = QuantizedDepth[CurrentFrameIndex].data();
    }

    EncodeZeroes(width, height, depth);

    CompressImage(width, height, depth, prev_depth);

    // Do Zstd compressions all together to keep the code cache hot:

    Pad12(Surfaces);
    Pack12(Surfaces, Packed);
    Surfaces_UncompressedBytes = static_cast<unsigned>( Packed.size() );
    ZstdCompress(Packed, SurfacesOut);

    Pad12(Edges);
    Pack12(Edges, Packed);
    Edges_UncompressedBytes = static_cast<unsigned>( Packed.size() );
    ZstdCompress(Packed, EdgesOut);

    Zeroes_UncompressedBytes = static_cast<unsigned>( Zeroes.size() );
    ZstdCompress(Zeroes, ZeroesOut);

    Blocks_UncompressedBytes = static_cast<unsigned>( Blocks.size() );
    ZstdCompress(Blocks, BlocksOut);

    WriteCompressedFile(width, height, keyframe, compressed);

    return DepthResult::Success;
}

void DepthCompressor::CompressImage(
    int width,
    int height,
    const uint16_t* depth,
    const uint16_t* prev_depth)
{
    // Accumulated through the end of the filtering then compressed separately
    Surfaces.clear();
    Blocks.clear();
    Edges.clear();

    const int cy = height / kBlockSize;
    const int cx = width / kBlockSize;
    Blocks.resize((cx-1) * (cy-1));

    const uint16_t* outer_row = depth;
    for (int iy = 0; iy < cy; ++iy, outer_row += width * kBlockSize)
    {
        const uint16_t* inner_row = outer_row;

        for (int ix = 0; ix < cx; ++ix, inner_row += kBlockSize)
        {
            if (ix == 0 || iy == 0)
            {
                const uint16_t* row = inner_row;
                for (int y = 0; y < kBlockSize; ++y, row += width)
                {
                    for (int x = 0; x < kBlockSize; ++x)
                    {
                        // Get quantized depth value for this element
                        const unsigned d = row[x];
                        if (d == 0) {
                            continue;
                        }

                        const unsigned left0 = x > 0 ? row[x - 1] : 0;
                        const unsigned up0 = y > 0 ? row[x - width] : 0;
                        const unsigned zigzag = ApplyPrediction(d, Predict_Larger(left0, up0));
                        if (!left0 || !up0) {
                            Edges.push_back(static_cast<uint16_t>( zigzag ));
                        } else {
                            Surfaces.push_back(static_cast<uint16_t>( zigzag ));
                        }
                    }
                }

                continue;
            }

            // Identify the prediction with the best accuracy for this block:
            unsigned pred_sum[PredictorType_Count] = {0};

            const uint16_t* row = inner_row;
            for (int y = 0; y < kBlockSize; ++y, row += width)
            {
                for (int x = 0; x < kBlockSize; ++x)
                {
                    const unsigned d = row[x];
                    if (d != 0)
                    {
                        const unsigned left0 = row[x - 1];
                        const unsigned left1 = row[x - 2];
                        const unsigned up0 = row[x - width];
                        const unsigned up1 = row[x - width * 2];

                        pred_sum[PredictorType_Larger] += ApplyPrediction(d, Predict_Larger(left0, up0));
                        pred_sum[PredictorType_Up] += ApplyPrediction(d, Predict_Up(left0, up0));
                        pred_sum[PredictorType_Left] += ApplyPrediction(d, Predict_Left(left0, up0));
                        pred_sum[PredictorType_UpTrend] += ApplyPrediction(d, Predict_UpTrend(left0, up0, up1));
                        pred_sum[PredictorType_LeftTrend] += ApplyPrediction(d, Predict_LeftTrend(left0, left1, up0));
                        pred_sum[PredictorType_Average] += ApplyPrediction(d, Predict_Average(left0, up0));

                        if (prev_depth) {
                            pred_sum[PredictorType_PrevFrame] += ApplyPrediction(d, Predict_PrevFrame(prev_depth[row + x - depth], left0, up0));
                        }
                    }
                }
            } // end x,y loop

            // Find best one
            unsigned smallest_i = pred_sum[0];
            int best_predictor = 0;
            int pred_count = PredictorType_Count;
            if (!prev_depth) {
                pred_count = PredictorType_PrevFrame; // Skip P-frame ones
            }
            for (int i = 1; i < pred_count; ++i)
            {
                if (pred_sum[i] < smallest_i) {
                    best_predictor = i;
                    smallest_i = pred_sum[i];
                }
            }
            Blocks[(iy-1) * (cx-1) + (ix-1)] = static_cast<uint8_t>( best_predictor );

            row = inner_row;
            for (int y = 0; y < kBlockSize; ++y, row += width)
            {
                for (int x = 0; x < kBlockSize; ++x)
                {
                    const unsigned d = row[x];
                    if (d == 0) {
                        continue;
                    }

                    const unsigned left0 = row[x - 1];
                    const unsigned up0 = row[x - width];

                    unsigned zigzag;
                    switch (best_predictor)
                    {
                    default:
                    case PredictorType_Larger:
                        zigzag = ApplyPrediction(d, Predict_Larger(left0, up0));
                        break;
                    case PredictorType_Up:
                        zigzag = ApplyPrediction(d, Predict_Up(left0, up0));
                        break;
                    case PredictorType_Left:
                        zigzag = ApplyPrediction(d, Predict_Left(left0, up0));
                        break;
                    case PredictorType_UpTrend:
                        zigzag = ApplyPrediction(d, Predict_UpTrend(left0, up0, row[x - width * 2]));
                        break;
                    case PredictorType_LeftTrend:
                        zigzag = ApplyPrediction(d, Predict_LeftTrend(left0, row[x - 2], up0));
                        break;
                    case PredictorType_Average:
                        zigzag = ApplyPrediction(d, Predict_Average(left0, up0));
                        break;
                    case PredictorType_PrevFrame:
                        zigzag = ApplyPrediction(d, Predict_PrevFrame(prev_depth[row + x - depth], left0, up0));
                        break;
                    }
                    if (!left0 || !up0) {
                        Edges.push_back(static_cast<uint16_t>( zigzag ));
                    } else {
                        Surfaces.push_back(static_cast<uint16_t>( zigzag ));
                    }
                }
            } // next depth pixel
        }
    } // next block
}

void DepthCompressor::WriteCompressedFile(
    int width,
    int height,
    bool keyframe,
    std::vector<uint8_t>& compressed)
{
    compressed.resize(
        kDepthHeaderBytes +
        ZeroesOut.size() +
        SurfacesOut.size() +
        BlocksOut.size() +
        EdgesOut.size());
    uint8_t* copy_dest = compressed.data();

    // Write header
    copy_dest[0] = kDepthFormatMagic;

    uint8_t flags = 0;
    if (keyframe) {
        flags |= 1;
    }
    copy_dest[1] = flags;

    WriteU16_LE(copy_dest + 2, static_cast<uint16_t>( CompressedFrameNumber ));
    WriteU16_LE(copy_dest + 4, static_cast<uint16_t>( width ));
    WriteU16_LE(copy_dest + 6, static_cast<uint16_t>( height ));
    WriteU32_LE(copy_dest + 8, Zeroes_UncompressedBytes);
    WriteU32_LE(copy_dest + 12, static_cast<uint32_t>( ZeroesOut.size() ));
    WriteU32_LE(copy_dest + 16, Blocks_UncompressedBytes);
    WriteU32_LE(copy_dest + 20, static_cast<uint32_t>( BlocksOut.size() ));
    WriteU32_LE(copy_dest + 24, Edges_UncompressedBytes);
    WriteU32_LE(copy_dest + 28, static_cast<uint32_t>( EdgesOut.size() ));
    WriteU32_LE(copy_dest + 32, Surfaces_UncompressedBytes);
    WriteU32_LE(copy_dest + 36, static_cast<uint32_t>( SurfacesOut.size() ));
    copy_dest += kDepthHeaderBytes;

    // Concatenate the compressed data
    memcpy(copy_dest, ZeroesOut.data(), ZeroesOut.size());
    copy_dest += ZeroesOut.size();
    memcpy(copy_dest, BlocksOut.data(), BlocksOut.size());
    copy_dest += BlocksOut.size();
    memcpy(copy_dest, EdgesOut.data(), EdgesOut.size());
    copy_dest += EdgesOut.size();
    memcpy(copy_dest, SurfacesOut.data(), SurfacesOut.size());
}

DepthResult DepthCompressor::Decompress(
    const std::vector<uint8_t>& compressed,
    int& width,
    int& height,
    std::vector<uint16_t>& depth_out)
{
    if (compressed.size() < kDepthHeaderBytes) {
        return DepthResult::FileTruncated;
    }
    const uint8_t* src = compressed.data();
    if (src[0] != kDepthFormatMagic) {
        return DepthResult::WrongFormat;
    }
    bool keyframe = (src[1] & 1) != 0;
    const unsigned frame_number = ReadU16_LE(src + 2);

    if (!keyframe && frame_number != CompressedFrameNumber + 1) {
        return DepthResult::MissingPFrame;
    }
    CompressedFrameNumber = frame_number;

    width = ReadU16_LE(src + 4);
    height = ReadU16_LE(src + 6);
    if (width < 1 || width > 4096 || height < 1 || height > 4096) {
        return DepthResult::Corrupted;
    }

    // Get depth for previous frame
    const int n = width * height;
    QuantizedDepth[CurrentFrameIndex].resize(n);
    uint16_t* depth = QuantizedDepth[CurrentFrameIndex].data();
    CurrentFrameIndex = (CurrentFrameIndex + 1) % 2;
    uint16_t* prev_depth = nullptr;
    if (!keyframe) {
        if (QuantizedDepth[CurrentFrameIndex].size() != static_cast<size_t>( n )) {
            return DepthResult::MissingPFrame;
        }
        prev_depth = QuantizedDepth[CurrentFrameIndex].data();
    }

    Zeroes_UncompressedBytes = ReadU32_LE(src + 8);
    const unsigned ZeroesCompressedBytes = ReadU32_LE(src + 12);
    Blocks_UncompressedBytes = ReadU32_LE(src + 16);
    const unsigned BlocksCompressedBytes = ReadU32_LE(src + 20);
    Edges_UncompressedBytes = ReadU32_LE(src + 24);
    const unsigned EdgesCompressedBytes = ReadU32_LE(src + 28);
    Surfaces_UncompressedBytes = ReadU32_LE(src + 32);
    const unsigned SurfacesCompressedBytes = ReadU32_LE(src + 36);

    if (Blocks_UncompressedBytes < 2) {
        return DepthResult::Corrupted;
    }

    if (compressed.size() !=
        kDepthHeaderBytes +
        ZeroesCompressedBytes +
        BlocksCompressedBytes +
        EdgesCompressedBytes +
        SurfacesCompressedBytes)
    {
        return DepthResult::FileTruncated;
    }

    const uint8_t* ZeroesData = src + kDepthHeaderBytes;
    const uint8_t* BlocksData = ZeroesData + ZeroesCompressedBytes;
    const uint8_t* EdgesData = BlocksData + BlocksCompressedBytes;
    const uint8_t* SurfacesData = EdgesData + EdgesCompressedBytes;

    bool success = ZstdDecompress(
        ZeroesData,
        ZeroesCompressedBytes,
        Zeroes_UncompressedBytes,
        Zeroes);
    if (!success) {
        return DepthResult::Corrupted;
    }

    success = ZstdDecompress(
        EdgesData,
        EdgesCompressedBytes,
        Edges_UncompressedBytes,
        Packed);
    if (!success) {
        return DepthResult::Corrupted;
    }
    Unpack12(Packed, Edges);

    success = ZstdDecompress(
        SurfacesData,
        SurfacesCompressedBytes,
        Surfaces_UncompressedBytes,
        Packed);
    if (!success) {
        return DepthResult::Corrupted;
    }
    Unpack12(Packed, Surfaces);

    success = ZstdDecompress(
        BlocksData,
        BlocksCompressedBytes,
        Blocks_UncompressedBytes,
        Blocks);
    if (!success) {
        return DepthResult::Corrupted;
    }

    if (Zeroes.size() != static_cast<size_t>( n ) / 8) {
        return DepthResult::Corrupted;
    }
    DecodeZeroes(width, height, depth);

    success = DecompressImage(width, height, depth, prev_depth);
    if (!success) {
        return DepthResult::Corrupted;
    }

    DequantizeDepthImage(width, height, depth, depth_out);
    return DepthResult::Success;
}

bool DepthCompressor::DecompressImage(
    int width,
    int height,
    uint16_t* depth,
    const uint16_t* prev_depth)
{
    const int cy = height / kBlockSize;
    const int cx = width / kBlockSize;
    if (Blocks.size() != static_cast<size_t>( (cx - 1) * (cy - 1) )) {
        return false;
    }

    unsigned EdgesIndex = 0, SurfacesIndex = 0;

    uint16_t* outer_row = depth;
    for (int iy = 0; iy < cy; ++iy, outer_row += kBlockSize * width)
    {
        uint16_t* inner_row = outer_row;
        for (int ix = 0; ix < cx; ++ix, inner_row += kBlockSize)
        {
            if (ix == 0 || iy == 0)
            {
                uint16_t* row = inner_row;
                for (int y = 0; y < kBlockSize; ++y, row += width)
                {
                    for (int x = 0; x < kBlockSize; ++x)
                    {
                        // Get quantized depth value for this element
                        unsigned d = row[x];
                        if (d == 0) {
                            continue;
                        }

                        const unsigned left0 = x > 0 ? row[x - 1] : 0;
                        const unsigned up0 = y > 0 ? row[x - width] : 0;

                        unsigned zigzag;
                        if (!left0 || !up0) {
                            if (EdgesIndex >= Edges.size()) {
                                return false;
                            }
                            zigzag = Edges[EdgesIndex++];
                        } else {
                            if (SurfacesIndex >= Surfaces.size()) {
                                return false;
                            }
                            zigzag = Surfaces[SurfacesIndex++];
                        }

                        d = UndoPrediction(zigzag, Predict_Larger(left0, up0));
                        row[x] = static_cast<uint16_t>( d );
                    }
                }

                continue;
            }

            const uint8_t predictor = Blocks[(iy-1) * (cx-1) + (ix-1)];

            uint16_t* row = inner_row;
            for (int y = 0; y < kBlockSize; ++y, row += width)
            {
                for (int x = 0; x < kBlockSize; ++x)
                {
                    unsigned d = row[x];
                    if (d == 0) {
                        continue;
                    }

                    const unsigned left0 = row[x - 1];
                    const unsigned up0 = row[x - width];

                    unsigned zigzag;
                    if (!left0 || !up0) {
                        if (EdgesIndex >= Edges.size()) {
                            return false;
                        }
                        zigzag = Edges[EdgesIndex++];
                    } else {
                        if (SurfacesIndex >= Surfaces.size()) {
                            return false;
                        }
                        zigzag = Surfaces[SurfacesIndex++];
                    }

                    switch (predictor)
                    {
                    default:
                    case PredictorType_Larger:
                        d = UndoPrediction(zigzag, Predict_Larger(left0, up0));
                        break;
                    case PredictorType_Up:
                        d = UndoPrediction(zigzag, Predict_Up(left0, up0));
                        break;
                    case PredictorType_Left:
                        d = UndoPrediction(zigzag, Predict_Left(left0, up0));
                        break;
                    case PredictorType_UpTrend:
                        d = UndoPrediction(zigzag, Predict_UpTrend(left0, up0, row[x - width * 2]));
                        break;
                    case PredictorType_LeftTrend:
                        d = UndoPrediction(zigzag, Predict_LeftTrend(left0, row[x - 2], up0));
                        break;
                    case PredictorType_Average:
                        d = UndoPrediction(zigzag, Predict_Average(left0, up0));
                        break;
                    case PredictorType_PrevFrame:
                        d = UndoPrediction(zigzag, Predict_PrevFrame(prev_depth[row + x - depth], left0, up0));
                        break;
                    }

                    row[x] = static_cast<uint16_t>( d );
                }
            } // next depth pixel
        }
    } // next block

    return true;
}

void DepthCompressor::EncodeZeroes(
    int width,
    int height,
    const uint16_t* depth)
{
    const int bytes = width * height / 8;
    Zeroes.resize(bytes);

    unsigned prev = 0;
    for (int i = 0; i < bytes; ++i, depth += 8)
    {
        unsigned bits = 0;
        for (int j = 0; j < 8; ++j) {
            const unsigned x = depth[j] ? 1 : 0;
            bits |= (x ^ prev) << j;
            prev = x;
        }

        Zeroes[i] = static_cast<uint8_t>( bits );
    }
}

void DepthCompressor::DecodeZeroes(
    int width,
    int height,
    uint16_t* depth)
{
    const int bytes = width * height / 8;

    unsigned prev = 0;
    for (int i = 0; i < bytes; ++i, depth += 8)
    {
        const unsigned bits = Zeroes[i];

        for (int j = 0; j < 8; ++j) {
            unsigned x_xor_prev = (bits >> j) & 1;
            prev ^= x_xor_prev;
            depth[j] = static_cast<uint16_t>( prev );
        }
    }
}


} // namespace zdepth
