#ifndef __EUSPCL_PCL_UTIL_H__
#define __EUSPCL_PCL_UTIL_H__

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
