#include "viewport.h"

#include "CMU462.h"

namespace CMU462 {

void ViewportImp::set_viewbox( float centerX, float centerY, float vspan ) {

  // Task 5 (part 2): 
  // Set svg coordinate to normalized device coordinate transformation. Your input
  // arguments are defined as normalized SVG canvas coordinates.
  this->centerX = centerX;
  this->centerY = centerY;
  this->vspan = vspan;

  double translation_data[]{
      1.0, 0.0, 0.5 - centerX * 0.5 / vspan,
      0.0, 1.0, 0.5 - centerY * 0.5 / vspan,
      0.0, 0.0, 1.0};
  Matrix3x3 translation(translation_data);
  double scale_data[]{
      0.5 / vspan, 0.0, 0.0,
      0.0, 0.5 / vspan, 0.0,
      0.0, 0.0, 1.0
  };
  Matrix3x3 scale(scale_data);
  set_svg_2_norm(translation * scale);
}

void ViewportImp::update_viewbox( float dx, float dy, float scale ) { 
  
  this->centerX -= dx;
  this->centerY -= dy;
  this->vspan *= scale;
  set_viewbox( centerX, centerY, vspan );
}

} // namespace CMU462
