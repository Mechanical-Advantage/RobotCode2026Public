// Copyright (c) 2022-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "metal_context.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <iostream>

static const char *SHADER_SRC = R"(
#include <metal_stdlib>
using namespace metal;

struct Constants {
    int width;
    int height;
    int window_size;
    int threshold_val;
};

// Pass 1: Horizontal Sum
kernel void box_filter_x(
    device const uint8_t* inBuffer [[buffer(0)]],
    device uint16_t* outBuffer [[buffer(1)]],
    constant Constants& params [[buffer(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    if (gid.x >= (uint)params.width || gid.y >= (uint)params.height) return;

    int radius = params.window_size / 2;
    int sum = 0;
    
    for (int dx = -radius; dx <= radius; ++dx) {
        int px = clamp((int)gid.x + dx, 0, params.width - 1);
        sum += inBuffer[gid.y * params.width + px];
    }
    
    outBuffer[gid.y * params.width + gid.x] = sum;
}

// Pass 2: Vertical Sum and Threshold
kernel void box_filter_y_and_threshold(
    device const uint16_t* inBuffer [[buffer(0)]],
    device const uint8_t* originalImage [[buffer(1)]],
    device uint8_t* outBuffer [[buffer(2)]],
    constant Constants& params [[buffer(3)]],
    uint2 gid [[thread_position_in_grid]]
) {
    if (gid.x >= (uint)params.width || gid.y >= (uint)params.height) return;

    int radius = params.window_size / 2;
    int sum = 0;
    
    for (int dy = -radius; dy <= radius; ++dy) {
        int py = clamp((int)gid.y + dy, 0, params.height - 1);
        sum += inBuffer[py * params.width + gid.x];
    }
    
    int area = params.window_size * params.window_size;
    int avg_uint8 = sum / area; // Truncation
    
    int center_uint8 = originalImage[gid.y * params.width + gid.x];
    
    int diff = avg_uint8 - center_uint8;
    if (diff < 0) diff = 0;
    
    uint8_t out_val = (diff > params.threshold_val) ? 255 : 0;
    
    outBuffer[gid.y * params.width + gid.x] = out_val;
}
)";

class ArucoMaxMetalContext::Impl {
public:
  id<MTLDevice> device;
  id<MTLCommandQueue> commandQueue;
  id<MTLComputePipelineState> pso_x;
  id<MTLComputePipelineState> pso_y;

  id<MTLBuffer> inBuffer;
  id<MTLBuffer> intermediateBuffer;
  id<MTLBuffer> outBuffer;

  int current_width = 0;
  int current_height = 0;

  Impl() {
    device = MTLCreateSystemDefaultDevice();
    if (!device) {
      std::cerr << "Failed to find Metal device!" << std::endl;
      return;
    }

    commandQueue = [device newCommandQueue];

    NSError *error = nil;
    NSString *source = [NSString stringWithUTF8String:SHADER_SRC];
    MTLCompileOptions *options = [[MTLCompileOptions alloc] init];

    id<MTLLibrary> library =
        [device newLibraryWithSource:source options:options error:&error];
    if (!library) {
      std::cerr << "Failed to compile Metal shader: " <<
          [[error localizedDescription] UTF8String] << std::endl;
      return;
    }

    id<MTLFunction> funcX = [library newFunctionWithName:@"box_filter_x"];
    id<MTLFunction> funcY =
        [library newFunctionWithName:@"box_filter_y_and_threshold"];

    pso_x = [device newComputePipelineStateWithFunction:funcX error:&error];
    if (!pso_x)
      std::cerr << "Failed to create PSO X: " <<
          [[error localizedDescription] UTF8String] << std::endl;

    pso_y = [device newComputePipelineStateWithFunction:funcY error:&error];
    if (!pso_y)
      std::cerr << "Failed to create PSO Y: " <<
          [[error localizedDescription] UTF8String] << std::endl;
  }

  void allocateBuffers(int width, int height) {
    if (width == current_width && height == current_height && inBuffer != nil)
      return;

    current_width = width;
    current_height = height;

    int num_pixels = width * height;
    inBuffer = [device newBufferWithLength:num_pixels * sizeof(uint8_t)
                                   options:MTLResourceStorageModeShared];
    // Private mode for intermediate is faster on Apple Silicon
    intermediateBuffer =
        [device newBufferWithLength:num_pixels * sizeof(uint16_t)
                            options:MTLResourceStorageModePrivate];
    outBuffer = [device newBufferWithLength:num_pixels * sizeof(uint8_t)
                                    options:MTLResourceStorageModeShared];
  }
};

ArucoMaxMetalContext::ArucoMaxMetalContext() : pImpl(new Impl()) {}

ArucoMaxMetalContext::~ArucoMaxMetalContext() { delete pImpl; }

struct Constants {
  int width;
  int height;
  int window_size;
  int threshold_val;
};

uint8_t *ArucoMaxMetalContext::adaptiveThreshold(const uint8_t *inImage,
                                                 int width, int height,
                                                 int window_size,
                                                 int threshold_val) {
  if (!pImpl->device)
    return nullptr;

  pImpl->allocateBuffers(width, height);

  // Copy input data to Metal buffer
  memcpy([pImpl->inBuffer contents], inImage, width * height * sizeof(uint8_t));

  id<MTLCommandBuffer> commandBuffer = [pImpl->commandQueue commandBuffer];

  Constants params = {width, height, window_size, threshold_val};

  // Pass 1
  id<MTLComputeCommandEncoder> computeEncoderX =
      [commandBuffer computeCommandEncoder];
  [computeEncoderX setComputePipelineState:pImpl->pso_x];
  [computeEncoderX setBuffer:pImpl->inBuffer offset:0 atIndex:0];
  [computeEncoderX setBuffer:pImpl->intermediateBuffer offset:0 atIndex:1];
  [computeEncoderX setBytes:&params length:sizeof(Constants) atIndex:2];

  MTLSize gridSize = MTLSizeMake(width, height, 1);
  MTLSize threadgroupSize = MTLSizeMake(16, 16, 1);

  [computeEncoderX dispatchThreads:gridSize
             threadsPerThreadgroup:threadgroupSize];
  [computeEncoderX endEncoding];

  // Pass 2
  id<MTLComputeCommandEncoder> computeEncoderY =
      [commandBuffer computeCommandEncoder];
  [computeEncoderY setComputePipelineState:pImpl->pso_y];
  [computeEncoderY setBuffer:pImpl->intermediateBuffer offset:0 atIndex:0];
  [computeEncoderY setBuffer:pImpl->inBuffer
                      offset:0
                     atIndex:1]; // originalImage
  [computeEncoderY setBuffer:pImpl->outBuffer offset:0 atIndex:2];
  [computeEncoderY setBytes:&params length:sizeof(Constants) atIndex:3];

  [computeEncoderY dispatchThreads:gridSize
             threadsPerThreadgroup:threadgroupSize];
  [computeEncoderY endEncoding];

  [commandBuffer commit];
  [commandBuffer waitUntilCompleted];

  return (uint8_t *)[pImpl->outBuffer contents];
}
