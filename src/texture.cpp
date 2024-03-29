#include "texture.h"
#include "color.h"

#include <assert.h>
#include <iostream>
#include <algorithm>

using namespace std;

namespace CMU462 {

inline void uint8_to_float( float dst[4], unsigned char* src ) {
  uint8_t* src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
  dst[3] = src_uint8[3] / 255.f;
}

inline void float_to_uint8( unsigned char* dst, float src[4] ) {
  uint8_t* dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[0])));
  dst_uint8[1] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[1])));
  dst_uint8[2] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[2])));
  dst_uint8[3] = (uint8_t) ( 255.f * max( 0.0f, min( 1.0f, src[3])));
}

void Sampler2DImp::generate_mips(Texture& tex, int startLevel) {

  // NOTE: 
  // This starter code allocates the mip levels and generates a level 
  // map by filling each level with placeholder data in the form of a 
  // color that differs from its neighbours'. You should instead fill
  // with the correct data!

  // Task 7: Implement this

  // check start level
  if ( startLevel >= tex.mipmap.size() ) {
    std::cerr << "Invalid start level"; 
  }

  // allocate sublevels
  int baseWidth  = tex.mipmap[startLevel].width;
  int baseHeight = tex.mipmap[startLevel].height;
  int numSubLevels = (int)(log2f( (float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  tex.mipmap.resize(startLevel + numSubLevels + 1);

  int width  = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel& level = tex.mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width  = max( 1, width  / 2); assert(width  > 0);
    height = max( 1, height / 2); assert(height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(4 * width * height);

    const auto &lower_level = tex.mipmap[startLevel + i - 1];
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        auto x0 = x * 2;
        auto y0 = y * 2;
        auto x1 = x * 2 + 1;
        auto y1 = y * 2 + 1;
        for (int j = 0; j < 4; ++j) {
          int sum = 0;
          sum += lower_level.texels[4 * (x0 + y0 * lower_level.width) + j];
          sum += lower_level.texels[4 * (x1 + y0 * lower_level.width) + j];
          sum += lower_level.texels[4 * (x0 + y1 * lower_level.width) + j];
          sum += lower_level.texels[4 * (x1 + y1 * lower_level.width) + j];
          sum /= 4;
          sum = min(sum, 255);
          level.texels[4 * (x + y * level.width) + j] = sum;
        }
      }
    }
  }

  // fill all 0 sub levels with interchanging colors (JUST AS A PLACEHOLDER)
//  Color colors[3] = { Color(1,0,0,1), Color(0,1,0,1), Color(0,0,1,1) };
//  for(size_t i = 1; i < tex.mipmap.size(); ++i) {
//
//    Color c = colors[i % 3];
//    MipLevel& mip = tex.mipmap[i];
//
//    for(size_t i = 0; i < 4 * mip.width * mip.height; i += 4) {
//      float_to_uint8( &mip.texels[i], &c.r );
//    }
//  }

}

Color Sampler2DImp::sample_nearest(Texture& tex, 
                                   float u, float v, 
                                   int level) {

  // Task 6: Implement nearest neighbour interpolation

  if (level >= 0 && level < tex.mipmap.size()) {
    const auto &map = tex.mipmap[level];
    int su = floor(u * (float)map.width);
    int sv = floor(v * (float)map.height);
    return {(float)map.texels[4 * (su + sv * map.width)] / 255.0f,
            (float)map.texels[4 * (su + sv * map.width) + 1] / 255.0f,
            (float)map.texels[4 * (su + sv * map.width) + 2] / 255.0f,
            (float)map.texels[4 * (su + sv * map.width) + 3] / 255.0f
    };
  }
  
  // return magenta for invalid level
  return Color(1,0,1,1);

}

Color Sampler2DImp::sample_bilinear(Texture& tex, 
                                    float u, float v, 
                                    int level) {
  
  // Task 6: Implement bilinear filtering

  if (level >= 0 && level < tex.mipmap.size()) {
    const auto &map = tex.mipmap[level];
    int su0 = floor(u * (float)map.width - 0.5f);
    int sv0 = floor(v * (float)map.height - 0.5f);
    int su1 = su0 + 1;
    int sv1 = sv0 + 1;
    auto s = u * (float)map.width - ((float)su0 + 0.5f);
    auto t = v * (float)map.height - ((float)sv0 + 0.5f);
    su0 = max(su0, 0);
    sv0 = max(sv0, 0);
    su1 = max(su1, 0);
    sv1 = max(sv1, 0);
    su0 = min(su0, int(map.width - 1));
    sv0 = min(sv0, int(map.height - 1));
    su1 = min(su1, int(map.width - 1));
    sv1 = min(sv1, int(map.height - 1));
    Color c00{(float)map.texels[4 * (su0 + sv0 * map.width)] / 255.0f,
              (float)map.texels[4 * (su0 + sv0 * map.width) + 1] / 255.0f,
              (float)map.texels[4 * (su0 + sv0 * map.width) + 2] / 255.0f,
              (float)map.texels[4 * (su0 + sv0 * map.width) + 3] / 255.0f};
    Color c10{(float)map.texels[4 * (su1 + sv0 * map.width)] / 255.0f,
              (float)map.texels[4 * (su1 + sv0 * map.width) + 1] / 255.0f,
              (float)map.texels[4 * (su1 + sv0 * map.width) + 2] / 255.0f,
              (float)map.texels[4 * (su1 + sv0 * map.width) + 3] / 255.0f};
    Color c01{(float)map.texels[4 * (su0 + sv1 * map.width)] / 255.0f,
              (float)map.texels[4 * (su0 + sv1 * map.width) + 1] / 255.0f,
              (float)map.texels[4 * (su0 + sv1 * map.width) + 2] / 255.0f,
              (float)map.texels[4 * (su0 + sv1 * map.width) + 3] / 255.0f};
    Color c11{(float)map.texels[4 * (su1 + sv1 * map.width)] / 255.0f,
              (float)map.texels[4 * (su1 + sv1 * map.width) + 1] / 255.0f,
              (float)map.texels[4 * (su1 + sv1 * map.width) + 2] / 255.0f,
              (float)map.texels[4 * (su1 + sv1 * map.width) + 3] / 255.0f};
    return (1.0f - t) * ((1.0f - s) * c00 + s * c10)
        + t * ((1.0f - s) * c01 + s * c11);
  }

  // return magenta for invalid level
  return Color(1,0,1,1);

}

Color Sampler2DImp::sample_trilinear(Texture& tex, 
                                     float u, float v, 
                                     float u_scale, float v_scale) {

  // Task 7: Implement trilinear filtering

  auto scale = min(u_scale, v_scale);
  auto d = log2(scale);
  auto w = d - floor(d);
  int l = min(static_cast<int>(floor(d)),
              static_cast<int>(tex.mipmap.size()) - 2);
  int step = floor(max(u_scale, v_scale) / scale + 0.5f);
  Color sum(0, 0, 0, 0);

  if (u_scale < v_scale) {
    for (int i = 0; i < step; ++i) {
      auto h0 =
          sample_bilinear(tex,
                          u,
                          v + static_cast<float>(i) * scale
                              / static_cast<float>(tex.width),
                          max(0, l));
      auto h1 =
          sample_bilinear(tex,
                          u,
                          v + static_cast<float>(i) * scale
                              / static_cast<float>(tex.width),
                          max(0, l + 1));
      sum += (1.0f - w) * h0 + w * h1;
    }
  } else {
    for (int i = 0; i < step; ++i) {
      auto h0 =
          sample_bilinear(tex,
                          u + static_cast<float>(i) * scale
                              / static_cast<float>(tex.height),
                          v,
                          max(0, l));
      auto h1 =
          sample_bilinear(tex,
                          u + static_cast<float>(i) * scale
                              / static_cast<float>(tex.height),
                          v,
                          max(0, l + 1));
      sum += (1.0f - w) * h0 + w * h1;
    }
  }
  return 1.0f / static_cast<float>(step) * sum;

  // return magenta for invalid level
//  return Color(1,0,1,1);

}

} // namespace CMU462
