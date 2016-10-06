#ifndef __EUSPCL_PCL_UTIL_H__
#define __EUSPCL_PCL_UTIL_H__

#if 0
// RGB type conversion
pcl::RGB pclrgb;
pclrgb.rgb = float_rgb;
pclrgb.rgba = uint32_rgba;
pclrgb.r = uint8_r;
pclrgb.r = uint8_g;
pclrgb.r = uint8_b;

inline void floatRGBTo (const float rgb, unsigned int &irgb) {
  const unsigned int i_ = *reinterpret_cast<const unsigned int *>(&rgb);
  irgb = i_;
}

inline void floatRGBTo (const float rgb,
                        unsigned char &r, unsigned char &g, unsigned char &b) {
  const unsigned int irgb = *reinterpret_cast<const unsigned int *>(&rgb);
  r = (irgb & 0x00FF0000) >> 16;
  g = (irgb & 0x0000FF00) >> 8;
  b = (irgb & 0x000000FF);
}

inline void floatRGBTo (const float rgb,
                        float &r, float &g, float &b) {
  const unsigned int irgb = *reinterpret_cast<const unsigned int *>(&rgb);
  r = ((irgb & 0x00FF0000) >> 16)/255.0;
  g = ((irgb & 0x0000FF00) >> 8)/255.0;
  b = ((irgb & 0x000000FF) >> 0)/255.0;
}

inline void toFloatRGB(const unsigned int irgb, float &rgb) {
  const float f_ = *reinterpret_cast<const float *>(&irgb);
  rgb = f_;
}

inline void toFloatRGB(const unsigned char r,
                       const unsigned char g,
                       const unsigned char b,
                       float &rgb) {
  unsigned int irgb;
  irgb = b | (g << 8) | (r << 16);
  rgb = *reinterpret_cast<float *>(&irgb);
}

inline void toFloatRGB(const float fr,
                       const float fg,
                       const float fb,
                       float &rgb) {
  unsigned char r,g,b;
  r = (fr * 255.0);
  g = (fg * 255.0);
  b = (fb * 255.0);
  unsigned int irgb;
  irgb = b | (g << 8) | (r << 16);
  rgb = *reinterpret_cast<float *>(&irgb);
}
#endif

template < typename PTYPE >
void stepPointCloud (__PCL_NS::PointCloud < PTYPE > &in_cloud,
                     __PCL_NS::PointCloud < PTYPE > &out_cloud,
                     bool remove_nan, bool keep_organized,
                     int xoffset, int yoffset, int xstep, int ystep) {

  size_t xlength = in_cloud.width;
  size_t ylength = in_cloud.height;
  __PCL_NS::DefaultPointRepresentation< PTYPE > pr;

  if (xstep >= 1 && ystep >= 1) {
    for (size_t ypos = yoffset; ypos < ylength; ypos += ystep) {
      for (size_t xpos = xoffset; xpos < xlength; xpos += xstep) {
        PTYPE *it = &(in_cloud.points[(ypos * xlength) + xpos]);
        if (remove_nan) {
          if (!pr.isValid (*it)) {
            if (keep_organized) {
              PTYPE pt;
              out_cloud.points.push_back(pt);
            }
          } else {
            out_cloud.points.push_back(*it);
          }
        } else {
          out_cloud.points.push_back(*it);
        }
      }
    }
    if (keep_organized) {
      out_cloud.width = 1 + ((xlength - xoffset -1) / xstep);
      out_cloud.height = 1 + ((ylength - yoffset -1) / ystep);
    } else {
      out_cloud.width = out_cloud.points.size();
      out_cloud.height = 1;
    }
  } else if (remove_nan) {
    for ( typename __PCL_NS::PointCloud< PTYPE >::const_iterator it = in_cloud.begin();
         it != in_cloud.end(); it++) {
      if (!pr.isValid (*it)) {
        if (keep_organized) {
          PTYPE pt;
          out_cloud.points.push_back(pt);
        }
      } else {
        out_cloud.points.push_back(*it);
      }
    }
    if (!keep_organized &&
        (out_cloud.points.size() != in_cloud.points.size())) {
      out_cloud.width = out_cloud.points.size();
      out_cloud.height = 1;
    }
  }
}

#endif
