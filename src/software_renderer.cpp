#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include "triangulation.h"

using namespace std;

namespace CMU462 {

constexpr static auto epsilon = numeric_limits<float>::epsilon();

inline bool inside_triangle(float x0, float y0,
                            float x1, float y1,
                            float x2, float y2,
                            float x3, float y3) {
  Vector2D AB(x1 - x0, y1 - y0);
  Vector2D BC(x2 - x1, y2 - y1);
  Vector2D CA(x0 - x2, y0 - y2);
  Vector2D AP(x3 - x0, y3 - y0);
  Vector2D BP(x3 - x1, y3 - y1);
  Vector2D CP(x3 - x2, y3 - y2);
  auto c0 = cross(AB, AP);
  auto c1 = cross(BC, BP);
  auto c2 = cross(CA, CP);
  if ((c0 > 0 && c1 > 0 && c2 > 0) || (c0 < 0 && c1 < 0 && c2 < 0)
      || abs(c0) < epsilon || abs(c1) < epsilon || abs(c2) < epsilon) {
    return true;
  } else {
    return false;
  }
}

inline bool inside_rectangle(float bx0, float by0,
                             float bx1, float by1,
                             float x, float y) {
  return (x >= bx0 && x <= bx1 && y >= by0 && y <= by1);
}

inline bool rectangle_segment_intersection(float bx0, float by0,
                                           float bx1, float by1,
                                           float x0, float y0,
                                           float x1, float y1) {

  if (abs(x1 - x0) > epsilon) {
    auto iy0 = y0 + (bx0 - x0) * (y1 - y0) / (x1 - x0);
    auto iy1 = y0 + (bx1 - x0) * (y1 - y0) / (x1 - x0);
    if ((iy0 >= by0 && iy0 <= by1) || (iy1 >= by0 && iy1 <= by1)) {
      return true;
    }
  }

  if (abs(y1 - y0) > epsilon) {
    auto ix0 = x0 + (by0 - y0) * (x1 - x0) / (y1 - y0);
    auto ix1 = x0 + (by1 - y0) * (x1 - x0) / (y1 - y0);
    if ((ix0 >= bx0 && ix0 <= bx1) || (ix1 >= bx0 && ix1 <= bx1)) {
      return true;
    }
  }

  return false;
}

inline bool rectangle_triangle_intersection(float bx0, float by0,
                                            float bx1, float by1,
                                            float x0, float y0,
                                            float x1, float y1,
                                            float x2, float y2) {
  if (inside_rectangle(bx0, by0, bx1, by1, x0, y0)
      || inside_rectangle(bx0, by0, bx1, by1, x1, y1)
      || inside_rectangle(bx0, by0, bx1, by1, x2, y2)
      || rectangle_segment_intersection(bx0, by0, bx1, by1, x0, y0, x1, y1)
      || rectangle_segment_intersection(bx0, by0, bx1, by1, x1, y1, x2, y2)
      || rectangle_segment_intersection(bx0, by0, bx1, by1, x2, y2, x0, y0)) {
    return true;
  } else {
    return false;
  }
}

// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = svg_2_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y--;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y--;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y++;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y++;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;
  w = target_w * sample_rate;
  h = target_h * sample_rate;
  sample_buffer.resize(w * h * 4);
  memset(&sample_buffer[0], 255, sample_buffer.size());
}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;
  set_sample_rate(sample_rate);
}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  auto transformation_stack = transformation;
  transformation = transformation * element->transform;

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
    case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

  transformation = transformation_stack;
}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {

  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );

}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  fill_pixel(sx, sy, color);
}

void SoftwareRendererImp::rasterize_line( float x0, float y0,
                                          float x1, float y1,
                                          Color color) {

  // Task 2: 
  // Implement line rasterization

  int sx0 = (int)floor(x0);
  int sx1 = (int)floor(x1);
  int sy0 = (int)floor(y0);
  int sy1 = (int)floor(y1);

  bool stepping_over_y = abs(sx0 - sx1) < abs(sy0 - sy1);
  if (stepping_over_y) {
    swap(sx0, sy0);
    swap(sx1, sy1);
  }

  if (sx0 > sx1) {
    swap(sx0, sx1);
    swap(sy0, sy1);
  }

  int dx = sx1 - sx0;
  int dy = sy1 - sy0;
  int sy = sy0;
  int eps = 0;

  for (int sx = sx0; sx <= sx1; ++sx) {
    if (stepping_over_y) {
      fill_pixel(sy, sx, color);
    } else {
      fill_pixel(sx, sy, color);
    }

    eps += dy;
    if (dy >= 0) {
      if ((eps << 1) >= dx) {
        sy++;
        eps -= dx;
      }
    } else {
      if ((eps << 1) <= -dx) {
        sy--;
        eps += dx;
      }
    }
  }
}

void SoftwareRendererImp::rasterize_triangle( float x0, float y0,
                                              float x1, float y1,
                                              float x2, float y2,
                                              Color color ) {
  // Task 3: 
  // Implement triangle rasterization

  x0 *= sample_rate;
  y0 *= sample_rate;
  x1 *= sample_rate;
  y1 *= sample_rate;
  x2 *= sample_rate;
  y2 *= sample_rate;

  // bounding box
  int bx0 = floor(min(x0, min(x1, x2)));
  int bx1 = floor(max(x0, max(x1, x2)));
  int by0 = floor(min(y0, min(y1, y2)));
  int by1 = floor(max(y0, max(y1, y2)));

  rasterize_triangle_box(bx0, by0, bx1, by1,
                         x0, y0, x1, y1, x2, y2,
                         color);
}

void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization

  x0 *= (float)sample_rate;
  y0 *= (float)sample_rate;
  x1 *= (float)sample_rate;
  y1 *= (float)sample_rate;

  auto image_width = x1 - x0;
  auto image_height = y1 - y0;

  int sx0 = max(0, static_cast<int>(floor(x0)));
  int sy0 = max(0, static_cast<int>(floor(y0)));
  int sx1 = min(w - 1, static_cast<int>(floor(x1)));
  int sy1 = min(h - 1, static_cast<int>(floor(y1)));

  for (auto sy = sy0; sy <= sy1; ++sy) {
    for (auto sx = sx0; sx <= sx1; ++sx) {
      auto u0 = ((float)sx - x0 + 0.5f) / image_width;
      auto v0 = ((float)sy - y0 + 0.5f) / image_height;
      auto u1 = ((float)sx - x0 + 1.5f) / image_width;
      auto v1 = ((float)sy - y0 + 1.5f) / image_height;
      auto u_scale = (u1 - u0) * float(tex.width);
      auto v_scale = (v1 - v0) * float(tex.height);
      fill_sample(sx,
                  sy,
                  sampler->sample_trilinear(tex,
                                            u0,
                                            v0,
                                            u_scale,
                                            v_scale));
    }
  }
}

// resolve samples to render target
void SoftwareRendererImp::resolve() {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".

  for (int sy = 0; sy < target_h; ++sy) {
    for (int sx = 0; sx < target_w; ++sx) {
      for (int i = 0; i < 4; ++i) {
        int sum = 0;
        // box
        int lx = sx * sample_rate;
        int rx = (sx + 1) * sample_rate;
        int uy = sy * sample_rate;
        int dy = (sy + 1) * sample_rate;

        for (int by = uy; by < dy; ++by) {
          for (int bx = lx; bx < rx; ++bx) {
            sum += sample_buffer[4 * (bx + by * w) + i];
          }
        }
        render_target[4 * (sx + sy * target_w) + i] =
            sum / (sample_rate * sample_rate);
      }
    }
  }
  memset(&sample_buffer[0], 255, sample_buffer.size());
}

void SoftwareRendererImp::fill_sample(int sx, int sy, const Color &c) {
  if ( sx < 0 || sx >= w ) return;
  if ( sy < 0 || sy >= h ) return;
  Color old_c{static_cast<float>(sample_buffer[4 * (sx + sy * w)]) / 255.0f,
              static_cast<float>(sample_buffer[4 * (sx + sy * w) + 1]) / 255.0f,
              static_cast<float>(sample_buffer[4 * (sx + sy * w) + 2]) / 255.0f,
              static_cast<float>(sample_buffer[4 * (sx + sy * w) + 3]) / 255.0f
  };
  Color new_c;

  old_c.r *= old_c.a;
  old_c.g *= old_c.a;
  old_c.b *= old_c.a;

  new_c.a = 1.0f - (1.0f - c.a) * (1.0f - old_c.a);
  new_c.r = ((1.0f - c.a) * old_c.r + c.r * c.a) / new_c.a;
  new_c.g = ((1.0f - c.a) * old_c.g + c.g * c.a) / new_c.a;
  new_c.b = ((1.0f - c.a) * old_c.b + c.b * c.a) / new_c.a;

  sample_buffer[4 * (sx + sy * w)] = (uint8_t)(new_c.r * 255);
  sample_buffer[4 * (sx + sy * w) + 1] = (uint8_t)(new_c.g * 255);
  sample_buffer[4 * (sx + sy * w) + 2] = (uint8_t)(new_c.b * 255);
  sample_buffer[4 * (sx + sy * w) + 3] = (uint8_t)(new_c.a * 255);
}

void SoftwareRendererImp::fill_pixel(int x, int y, const Color &c) {
  if (x < 0 || x >= target_w) return;
  if (y < 0 || y >= target_h) return;

  // box
  int lx = x * sample_rate;
  int rx = (x + 1) * sample_rate;
  int uy = y * sample_rate;
  int dy = (y + 1) * sample_rate;

  for (int by = uy; by < dy; ++by) {
    for (int bx = lx; bx < rx; ++bx) {
      fill_sample(bx, by, c);
    }
  }
}

void SoftwareRendererImp::rasterize_triangle_box(int bx0, int by0,
                                                 int bx1, int by1,
                                                 float x0, float y0,
                                                 float x1, float y1,
                                                 float x2, float y2,
                                                 Color color) {
  // rasterize triangle for a box

  constexpr static int smallest_box_size = 8;

  if (bx0 >= w || bx1 < 0 || by0 >= h || by1 < 0) return;

  bool left_up_inside = inside_triangle(x0, y0, x1, y1, x2, y2,
                                        (float)bx0 + 0.5f, (float)by0 + 0.5f);
  bool right_up_inside = inside_triangle(x0, y0, x1, y1, x2, y2,
                                         (float)bx1 + 0.5f, (float)by0 + 0.5f);
  bool left_bottom_inside = inside_triangle(x0, y0, x1, y1, x2, y2,
                                            (float)bx0 + 0.5f,
                                            (float)by1 + 0.5f);
  bool right_bottom_inside = inside_triangle(x0, y0, x1, y1, x2, y2,
                                             (float)bx1 + 0.5f,
                                             (float)by1 + 0.5f);

  if (left_up_inside && right_up_inside && left_bottom_inside
      && right_bottom_inside) {
    for (int sy = by0; sy <= by1; ++sy) {
      for (int sx = bx0; sx <= bx1; ++sx) {
        fill_sample(sx, sy, color);
      }
    }
  } else if (!rectangle_triangle_intersection((float)bx0 + 0.5f,
                                              (float)by0 + 0.5f,
                                              (float)bx1 + 0.5f,
                                              (float)by1 + 0.5f,
                                              x0, y0, x1, y1, x2, y2)) {
    return;
  } else if (bx1 - bx0 <= smallest_box_size && by1 - by0 <= smallest_box_size) {
    for (int sy = by0; sy <= by1; ++sy) {
      for (int sx = bx0; sx <= bx1; ++sx) {
        if (inside_triangle(x0, y0, x1, y1, x2, y2,
                            (float)sx + 0.5f, (float)sy + 0.5f)) {
          fill_sample(sx, sy, color);
        }
      }
    }
  } else if (bx1 - bx0 >= by1 - by0) {
    int bx_center = bx0 + (bx1 - bx0) / 2;
    rasterize_triangle_box(bx0, by0, bx_center, by1,
                           x0, y0, x1, y1, x2, y2,
                           color);
    rasterize_triangle_box(bx_center + 1, by0, bx1, by1,
                           x0, y0, x1, y1, x2, y2,
                           color);
  } else {
    int by_center = by0 + (by1 - by0) / 2;
    rasterize_triangle_box(bx0, by0, bx1, by_center,
                           x0, y0, x1, y1, x2, y2,
                           color);
    rasterize_triangle_box(bx0, by_center + 1, bx1, by1,
                           x0, y0, x1, y1, x2, y2,
                           color);
  }
}

} // namespace CMU462
