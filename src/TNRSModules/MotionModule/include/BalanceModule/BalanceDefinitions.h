#pragma once

#include "MotionModule/include/MTypeHeader.h"

static const MType balanceDefs[2][24] =
  {
    //{ 0.0, 0.104666667, 1.57, 0.2007, -1.57, 0.0, -1.57, 1.57, -0.2007, 1.57, 0.0, 1.57, 0.0, -0.13434, -0.52674, 0.94754, -0.38327, 0.27297, 0.0, -0.11987, -0.33824, 0.59254, -0.21712, 0.25848},
    {
      0.0, 0.104666667,
      1.57, 0.2007, -1.57, 0.0, -1.57,
      1.57, -0.2007, 1.57, 0.0, 1.57,
      0.0, -0.17359, -0.63443, 1.24215, -0.57125, 0.28955,
      0.0, -0.15909, -0.52325, 1.03655, -0.47665, 0.27506
    },
    /*{
        0.0, 0.104666667,
        1.57, 0.2007, -1.57, 0.0, -1.57,
        1.57, -0.2007, 1.57, 0.0, 1.57,
        0.00000,  -0.31451,  -0.44925,   0.81786,  -0.36826,   0.33824,
        0.00000,  -0.31486,  -0.45047,   0.82065,  -0.36966,   0.33859
      },*/
    //{ 0.0, 0.104666667, 1.57, 0.2007, -1.57, 0.0, -1.57, 1.57, -0.2007, 1.57, 0.0, 1.57, 0.0, 0.15708, -1.0472 / 2, 1.0472, -1.0472 / 2, -0.31416, 0.0, 0.13963, -1.2217 / 2, 1.2217, -1.2217 / 2, -0.27925 }
    { 0.0, 0.104666667, 1.57, 0.2007, -1.57, 0.0, -1.57, 1.57, -0.2007, 1.57, 0.0, 1.57, 0.0, 0.15708, -1.0472 / 2, 1.0472, -1.0472 / 2, -0.31416, 0.0, 0.13963, -1.2217 / 2, 1.2217, -1.2217 / 2, -0.27925 }
    //{ 0.0, 0.104666667, 1.57, 0.2007, -1.57, 0.0, 1.57, 1.57, -0.2007, 1.57, 0.0, -1.57, 0.0, -0.11113, -0.49480, 0.93672, -0.46897, 0.25796, 0.0, -9.8629e-02, -3.0805e-01, 5.4629e-01, -2.6477e-01, 2.4539e-01},
    //{ 0.0, 0.104666667, 1.57, 0.2007, -1.57, 0.0, 1.57, 1.57, -0.2007, 1.57, 0.0, -1.57, 0.0, 9.8629e-02, -3.0805e-01, 5.4629e-01, -2.6477e-01, -2.4539e-01, 0.0, 0.11113, -0.49480, 0.93672, -0.46897, -0.25796}
    // Left Balance
    //{ 0.0, 0.27925, 1.57, 0.2007, -1.57, 0.0, 1.57, 1.57, -0.2007, 1.57, 0.0, -1.57, 0.0, -0.0037594, -0.4557055, 0.9358455, -0.5810201, 0.2174680, 0.0, 0.0073077, -0.0043197, 0.0194953, -0.1159248, 0.2064725},
    // Right Balance
    //{ 0.0, 0.27925, 1.57, 0.2007, -1.57, 0.0, 1.57, 1.57, -0.2007, 1.57, 0.0, -1.57, 0.0, -0.0073077, -0.0043197, 0.0194953, -0.1159248, -0.2064725, 0.0, 0.0037594, -0.4557055, 0.9358455, -0.5810201, -0.2174680}
  };
