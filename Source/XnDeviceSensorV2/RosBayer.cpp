/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
 *    Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *    William Morris <morris@ee.ccny.cuny.edu>
 *    Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Mihelich <mihelich@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <XnOpenNI.h>

// From ROS ni: ...\openni_camera\include\openni_camera\openni.h 
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )

// From ROS ni: ...\openni_camera\src\openni_driver.cpp
void bayer2RGB ( const XnUInt8 *bayer, XnUChar *image, XnUInt32 xres, XnUInt32 yres )
{
    register const XnUInt8 *bayer_pixel = bayer;
    register unsigned yIdx, xIdx;

    int line_step = xres;
    int line_step2 = xres << 1;

    int rgb_line_step  = line_step * 3;             // previous color line
    register unsigned char *rgb_pixel = (unsigned char *)image;

	// "OpenNI_EdgeAware" debayering method
    int dh, dv;

    // first two pixel values for first two lines
    // Bayer         0 1 2
    //         0     G r g
    // line_step     b g b
    // line_step2    g r g

    rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //         0     g R g
    // line_step     b g b
    // line_step2    g r g
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

    // BGBG line
    // Bayer         0 1 2
    //         0     g r g
    // line_step     B g b
    // line_step2    g r g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // pixel (1, 1)  0 1 2
    //         0     g r g
    // line_step     b G b
    // line_step2    g r g
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the first two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer        -1 0 1
    //           0   r G r
    //   line_step   g b g
    // line_step2    r g r
    rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

    // Bayer        -1 0 1
    //          0    r g R
    //  line_step    g b g
    // line_step2    r g r
    rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
    //rgb_pixel[5] = bayer_pixel[line_step];

    // BGBG line
    // Bayer        -1 0 1
    //          0    r g r
    //  line_step    g B g
    // line_step2    r g r
    rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
    rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         -1 0 1
    //         0      r g r
    // line_step      g b G
    // line_step2     r g r
    rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

    bayer_pixel += line_step + 2;
    rgb_pixel += rgb_line_step + 6;
    // main processing
    for (yIdx = 2; yIdx < yres-2; yIdx += 2)
    {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
            rgb_pixel[1] = bayer_pixel[0];
            rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g

            dh = abs( bayer_pixel[0] - bayer_pixel[2] );
            dv = abs( bayer_pixel[-line_step+1] - bayer_pixel[line_step+1] );

            if( dh > dv )
                rgb_pixel[4] = AVG( bayer_pixel[-line_step+1], bayer_pixel[line_step+1] );
            else if( dv > dh )
                rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[2] );
            else
                rgb_pixel[4] = AVG4( bayer_pixel[-line_step+1], bayer_pixel[line_step+1], bayer_pixel[0], bayer_pixel[2] );

            rgb_pixel[3] = bayer_pixel[1];
            rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
            rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

            dv = abs( bayer_pixel[0] - bayer_pixel[line_step2] );
            dh = abs( bayer_pixel[line_step-1] - bayer_pixel[line_step+1] );

            if( dv > dh )
                rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
            else if( dh > dv )
                rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0], bayer_pixel[line_step2] );
            else
                rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
            rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
            rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
    }

    //last two lines
    // Bayer         0 1 2
    //        -1     b g b
    //         0     G r g
    // line_step     b g b

    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g R g
    // line_step     b g b
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
    rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

    // BGBG line
    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     B g b
    //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     b G b
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the last two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r G r
    // line_step    g b g
    rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g R
    // line_step    g b g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
    //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

    // BGBG line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g B g
    //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g b G
    //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
}

void bayer2RGBWeighted ( const XnUInt8 *bayer, XnUChar *image, XnUInt32 xres, XnUInt32 yres )
{
    register const XnUInt8 *bayer_pixel = bayer;
    register unsigned yIdx, xIdx;

    int line_step = xres;
    int line_step2 = xres << 1;

    int rgb_line_step  = line_step * 3;             // previous color line
    register unsigned char *rgb_pixel = (unsigned char *)image;

	// "OpenNI_EdgeAwareWeighted" debayering method
    int dh, dv;

    // first two pixel values for first two lines
    // Bayer         0 1 2
    //         0     G r g
    // line_step     b g b
    // line_step2    g r g

    rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //         0     g R g
    // line_step     b g b
    // line_step2    g r g
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

    // BGBG line
    // Bayer         0 1 2
    //         0     g r g
    // line_step     B g b
    // line_step2    g r g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // pixel (1, 1)  0 1 2
    //         0     g r g
    // line_step     b G b
    // line_step2    g r g
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the first two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer        -1 0 1
    //           0   r G r
    //   line_step   g b g
    // line_step2    r g r
    rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

    // Bayer        -1 0 1
    //          0    r g R
    //  line_step    g b g
    // line_step2    r g r
    rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
    //rgb_pixel[5] = bayer_pixel[line_step];

    // BGBG line
    // Bayer        -1 0 1
    //          0    r g r
    //  line_step    g B g
    // line_step2    r g r
    rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
    rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         -1 0 1
    //         0      r g r
    // line_step      g b G
    // line_step2     r g r
    rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

    bayer_pixel += line_step + 2;
    rgb_pixel += rgb_line_step + 6;
    // main processing
    for (yIdx = 2; yIdx < yres-2; yIdx += 2)
    {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
            rgb_pixel[1] = bayer_pixel[0];
            rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g

            dh = abs( bayer_pixel[0] - bayer_pixel[2] );
            dv = abs( bayer_pixel[-line_step+1] - bayer_pixel[line_step+1] );

            if( dv == 0 && dh == 0 )
                rgb_pixel[4] = AVG4( bayer_pixel[1-line_step], bayer_pixel[1+line_step], bayer_pixel[0], bayer_pixel[2] );
            else
                rgb_pixel[4] = WAVG4( bayer_pixel[1-line_step], bayer_pixel[1+line_step], bayer_pixel[0], bayer_pixel[2], dh, dv );
            rgb_pixel[3] = bayer_pixel[1];
            rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
            rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

            dv = abs( bayer_pixel[0] - bayer_pixel[line_step2] );
            dh = abs( bayer_pixel[line_step-1] - bayer_pixel[line_step+1] );

            if( dv == 0 && dh == 0 )
                rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
            else
                rgb_pixel[rgb_line_step + 1] = WAVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1], dh, dv );

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
            rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
            rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
    }

    //last two lines
    // Bayer         0 1 2
    //        -1     b g b
    //         0     G r g
    // line_step     b g b

    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g R g
    // line_step     b g b
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
    rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

    // BGBG line
    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     B g b
    //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     b G b
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the last two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r G r
    // line_step    g b g
    rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g R
    // line_step    g b g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
    //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

    // BGBG line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g B g
    //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g b G
    //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
}

void bayer2RGBBilinear ( const XnUInt8 *bayer, XnUChar *image, XnUInt32 xres, XnUInt32 yres )
{
    register const XnUInt8 *bayer_pixel = bayer;
    register unsigned yIdx, xIdx;

    int line_step = xres;
    int line_step2 = xres << 1;

    int rgb_line_step  = line_step * 3;             // previous color line
    register unsigned char *rgb_pixel = (unsigned char *)image;

	// "OpenNI_Bilinear" debayering method
    // first two pixel values for first two lines
    // Bayer         0 1 2
    //         0     G r g
    // line_step     b g b
    // line_step2    g r g

    rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //         0     g R g
    // line_step     b g b
    // line_step2    g r g
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

    // BGBG line
    // Bayer         0 1 2
    //         0     g r g
    // line_step     B g b
    // line_step2    g r g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // pixel (1, 1)  0 1 2
    //         0     g r g
    // line_step     b G b
    // line_step2    g r g
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the first two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer        -1 0 1
    //           0   r G r
    //   line_step   g b g
    // line_step2    r g r
    rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

    // Bayer        -1 0 1
    //          0    r g R
    //  line_step    g b g
    // line_step2    r g r
    rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
    //rgb_pixel[5] = bayer_pixel[line_step];

    // BGBG line
    // Bayer        -1 0 1
    //          0    r g r
    //  line_step    g B g
    // line_step2    r g r
    rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
    rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         -1 0 1
    //         0      r g r
    // line_step      g b G
    // line_step2     r g r
    rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

    bayer_pixel += line_step + 2;
    rgb_pixel += rgb_line_step + 6;

    // main processing
    for (yIdx = 2; yIdx < yres-2; yIdx += 2)
    {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
            rgb_pixel[1] = bayer_pixel[0];
            rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g
            rgb_pixel[3] = bayer_pixel[1];
            rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
            rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
            rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
            rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
            rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
            rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
    }

    //last two lines
    // Bayer         0 1 2
    //        -1     b g b
    //         0     G r g
    // line_step     b g b

    rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
    rgb_pixel[1] = bayer_pixel[0];    // green pixel
    rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g R g
    // line_step     b g b
    //rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
    rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

    // BGBG line
    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     B g b
    //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer         0 1 2
    //        -1     b g b
    //         0     g r g
    // line_step     b G b
    //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

    rgb_pixel += 6;
    bayer_pixel += 2;
    // rest of the last two lines
    for (xIdx = 2; xIdx < xres - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
    {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
    }

    // last two pixel values for first two lines
    // GRGR line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r G r
    // line_step    g b g
    rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
    rgb_pixel[1] = bayer_pixel[0];
    rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g R
    // line_step    g b g
    rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
    rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
    //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

    // BGBG line
    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g B g
    //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
    rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
    rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

    // Bayer       -1 0 1
    //        -1    g b g
    //         0    r g r
    // line_step    g b G
    //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
    rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
    //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
}





