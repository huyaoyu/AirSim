//
// Created by yaoyu on 5/22/21.
//

// Standard headers.
#include <cmath>
#include <iostream>

// System headers.
#include <boost/math/constants/constants.hpp>

// Third party headers.
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Project headers.
#include "planar_2_full_view.h"

using namespace p2f;

const std::vector<std::function<std::pair<int, int>(int, int, int)>> p2f::ue_coor_2_ocv = {
        p2f::ue_pos_x_2_ocv, p2f::ue_neg_x_2_ocv,
        p2f::ue_pos_y_2_ocv, p2f::ue_neg_y_2_ocv,
        p2f::ue_pos_z_2_ocv, p2f::ue_neg_z_2_ocv
};

namespace p2f {
    FaceName unreal_engine_face_2_face_name(int ue_face) {
        FaceName face_name {FaceName::NEG_Z}; // The return value.
        switch (ue_face) {
            case 0:
            {
                // Positive x of UE4.
                face_name = FaceName::NEG_Z;
                break;
            }
            case 1:
            {
                // Negative x of UE4.
                face_name = FaceName::POS_Z;
                break;
            }
            case 2:
            {
                // Positive y of UE4.
                face_name = FaceName::NEG_X;
                break;
            }
            case 3:
            {
                // Negative y of UE4.
                face_name = FaceName::POS_X;
                break;
            }
            case 4:
            {
                // Positive z of UE4.
                face_name = FaceName::NEG_Y;
                break;
            }
            case 5:
            {
                // Negative z of UE4.
                face_name = FaceName::POS_Y;
                break;
            }
            default:
            {
                // Should never be here.
                std::cout << "Unexpected UE face id: " << ue_face << ". \n";
            }
        }

        return face_name;
    }
} // namespace p2f

/**
 * Normalize and clip an image.
 * @param mat Input image.
 * @param min_val Minimum value for normalization and clipping.
 * @param max_val Maximum value for normalization and clipping.
 * @return The normalized and clipped image.
 */
static cv::Mat make_single_channel_vis(const cv::Mat& mat, float min_val, float max_val) {
    // Covert the input mat to a float mat.
    cv::Mat temp;
    mat.convertTo(temp, CV_32FC1);

    // Normalize.
    temp = (temp - min_val) / ( max_val - min_val );

    // Clip by thresholding.
    cv::threshold( temp, temp, 1.0, 1.0, cv::THRESH_TRUNC );
    temp *= -1;
    cv::threshold( temp, temp, 0.0, 0.0, cv::THRESH_TRUNC );
    temp *= -255;

    // Convert to uint8.
    cv::Mat vis;
    temp.convertTo(vis, CV_8UC1);

    return vis;
}

Planar2FullView::Planar2FullView(int width, int out_h, int out_w)
: height(width), width(width)
, cross_height(height*3 + 2*H_PAD), cross_width(4*width + 2*W_PAD)
, out_h(out_h), out_w(out_w)
, image_cross(nullptr), map_x(nullptr), map_y(nullptr)
{
    // Allocate and initialize the memory for image_cross;
    const std::size_t cross_image_size = cross_height * cross_width;
    image_cross = new float[ cross_image_size ];
    std::memset(image_cross, 0.f, cross_image_size);

    // Create map_x and map_y.
    create_map_xy();
}

Planar2FullView::~Planar2FullView() {
    delete [] map_y; map_y = nullptr;
    delete [] map_x; map_x = nullptr;
    delete [] image_cross; image_cross = nullptr;
}

void Planar2FullView::make_padding() {
    // Refer to the following illustration for the locations of padding.
    // https://docs.google.com/presentation/d/1xfGt3oHgjvG_eGtKMlDg8MTYJhARNVKcTMcPl3s118o/edit?usp=sharing

    // 0.
    copy_by_stride(
            image_cross + (H_PAD + 3*height - 1) * cross_width + W_PAD + width, -cross_width,
            image_cross + (H_PAD + 2*height    ) * cross_width + W_PAD        ,  1,
            width);
    // 1.
    copy_by_stride(
            image_cross + (H_PAD + 2*height - 1) * cross_width + W_PAD + 4*width - 1, -1,
            image_cross + (H_PAD + 3*height    ) * cross_width + W_PAD +   width    ,  1,
            width);
    // 2.
    copy_by_stride(
            image_cross + (H_PAD + 2*height) * cross_width + W_PAD + 2*width - 1, cross_width,
            image_cross + (H_PAD + 2*height) * cross_width + W_PAD + 2*width    , 1,
            width);
    // 3.
    copy_by_stride(
            image_cross + (H_PAD + 3*height - 1) * cross_width + W_PAD + 2*width - 1, -1,
            image_cross + (H_PAD + 2*height    ) * cross_width + W_PAD + 3*width,  1,
            width);
    // 4.
    copy_by_stride(
            image_cross + (H_PAD + height) * cross_width + W_PAD + 3*width - 1, -1,
            image_cross + (H_PAD         ) * cross_width + W_PAD + 2*width, cross_width,
            width);
    // 5.
    copy_by_stride(
            image_cross + (H_PAD + height) * cross_width + W_PAD          , cross_width,
            image_cross + (H_PAD + height) * cross_width + W_PAD + 4*width, cross_width,
            width);
    // 6.
    copy_by_stride(
            image_cross + (H_PAD + 2*height - 1) * cross_width + W_PAD + 2*width, 1,
            image_cross + (H_PAD + 2*height    ) * cross_width + W_PAD + 2*width, cross_width,
            width);
}

// Padding = 1
// -->       +---------+
// |  x      |1,W+1    |1,2W+1
// v         |  neg_y  |
//  y        |   top   |
// +---------+---------+---------+---------+
// |H+1,1    |H+1,W+1  |H+1,2W+1 |H+1,3W+1 |H+1,4W+1
// |  pos_x  |  neg_z  |  neg_x  |  pos_z  |
// |   left  |  front  |  right  |         |
// +---------+---------+---------+---------+
//  2H+1,1   |2H+1,W+1 |2H+1,2W+1 2H+1,3W+1 2H+1,4W+1
//           |  pos_y  |
//           |  bottom |
//           +---------+
//           3H+1,W+1   3H+1,2W+1

void Planar2FullView::create_map_xy() {
    // Reset the memory.
    delete [] map_y;
    delete [] map_x;
    map_x = new float[ out_h * out_w ];
    map_y = new float[ out_h * out_w ];

    // Constants.
    const auto pi = boost::math::constants::pi<float>();
    const float one_fourth_pi   = pi / 4;
    const float three_fourth_pi = 3 * one_fourth_pi;
    const float five_fourth_pi  = 5 * one_fourth_pi;
    const float seven_fourth_pi = 7 * one_fourth_pi;
    const float two_pi          = 2 * pi;

    const float factor_lat = pi / (out_h-1);     // Allow the latitude angle to reach pi.
    const float factor_lon = two_pi / (out_w-1); // Allow the longitude angle to reach 2pi. This is by design.

    // The starting index of the individual faces in the image cross.
    const auto idx_pos_x = get_face_start_index(FaceName::POS_X);
    const auto idx_neg_x = get_face_start_index(FaceName::NEG_X);
    const auto idx_pos_y = get_face_start_index(FaceName::POS_Y);
    const auto idx_neg_y = get_face_start_index(FaceName::NEG_Y);
    const auto idx_pos_z = get_face_start_index(FaceName::POS_Z);
    const auto idx_neg_z = get_face_start_index(FaceName::NEG_Z);

    const int height_1 = height - 1;
    const int width_1  = width  - 1;

    // Loop for every pixel in the output image.
    for ( int i = 0; i < out_h; i++ ) {
        const float lat = i * factor_lat; // Latitude angle.
        const float y   = -std::cos(lat);
        const float sy  =  std::sin(lat); // Always positive.

        for ( int j = 0; j < out_w; j++ ) {
            const float lon = j * factor_lon; // Longitude angle.

            // xyz.
            const float x = -sy * std::cos(lon);
            const float z =  sy * std::sin(lon); // Use right-hand rule, so that the positive latitude direction is along the positive-y axis.

            // The absolute angles to the xy-plane and yz-plane.
            const float a_xy = pi - std::atan2( std::abs(z), y );
            const float a_yz = pi - std::atan2( std::abs(x), y );

            // The pixel coordinate in the image cross.
            float fx=0.f, fy=0.f;
            if ( a_xy < one_fourth_pi &&
                 a_yz < one_fourth_pi ) {
                // Top, neg_y.
                fx = ( 1 + x/y ) / 2 * width_1  + idx_neg_y.second;
                fy = ( 1 + z/y ) / 2 * height_1 + idx_neg_y.first;
            } else if ( a_xy >= three_fourth_pi &&
                        a_yz >= three_fourth_pi ) {
                // Bottom, pos_y.
                fx = ( 1 - x/y ) / 2 * width_1  + idx_pos_y.second;
                fy = ( 1 + z/y ) / 2 * height_1 + idx_pos_y.first;
            } else if ( lon < one_fourth_pi || lon >= seven_fourth_pi ) {
                // Right, neg_x.
                fx = ( 1 - z/x ) / 2 * width_1  + idx_neg_x.second;
                fy = ( 1 - y/x ) / 2 * height_1 + idx_neg_x.first;
            } else if ( one_fourth_pi <= lon && lon < three_fourth_pi ) {
                // Back, pos_z.
                fx = ( 1 + x/z ) / 2 * width_1  + idx_pos_z.second;
                fy = ( 1 + y/z ) / 2 * height_1 + idx_pos_z.first;
            } else if ( three_fourth_pi <= lon && lon < five_fourth_pi ) {
                // Left, pos_x.
                fx = ( 1 - z/x ) / 2 * width_1  + idx_pos_x.second;
                fy = ( 1 + y/x ) / 2 * height_1 + idx_pos_x.first;
            } else if ( five_fourth_pi <= lon && lon < seven_fourth_pi ) {
                // Front, neg_z.
                fx = ( 1 + x/z ) / 2 * width_1  + idx_neg_z.second;
                fy = ( 1 - y/z ) / 2 * height_1 + idx_neg_z.first;
            } else {
                // Should never be here.
                std::cout << "Wrong a_xy or z_yz. "
                          << "a_xy = " << a_xy
                          << "a_yz = " << a_yz << "\n";
            }

            map_x[i * out_w + j] = fx;
            map_y[i * out_w + j] = fy;
        }
    }
}

std::pair<int, int> Planar2FullView::get_face_start_index(const FaceName& face) const {
    std::pair<int, int> res; // H, W or y, x.

    switch ( face ) {
        case FaceName::POS_X:
        {
            res.first  = H_PAD + height;
            res.second = W_PAD;
            break;
        }
        case FaceName::NEG_X:
        {
            res.first  = H_PAD + height;
            res.second = W_PAD + 2*width;
            break;
        }
        case FaceName::POS_Y:
        {
            res.first  = H_PAD + 2*height;
            res.second = W_PAD + width;
            break;
        }
        case FaceName::NEG_Y:
        {
            res.first  = H_PAD;
            res.second = W_PAD + width;
            break;
        }
        case FaceName::POS_Z:
        {
            res.first  = H_PAD + height;
            res.second = W_PAD + 3*width;
            break;
        }
        case FaceName::NEG_Z:
        {
            res.first  = H_PAD + height;
            res.second = W_PAD + width;
            break;
        }
        default:{
            // Should never be here.
        }
    }
    return res;
}

void Planar2FullView::copy_by_stride(
        const float* from, int stride_from,
        float* to, int stride_to,
        int n) {
    for ( int i = 0; i < n; i++ ) {
        to[ i*stride_to ] = from[ i*stride_from ];
    }
}

bool Planar2FullView::write_image_cross_vis(const std::string& fn, float min_val, float max_val) const {
    // Create a cv::Mat wrap.
    cv::Mat wrap = cv::Mat(cross_height, cross_width, CV_32FC1, image_cross);

    // Create a visualization version of the cross image.
    cv::Mat vis = make_single_channel_vis(wrap, min_val, max_val);

    // Write.
    return cv::imwrite(fn, vis);
}

void Planar2FullView::sample( float* out ) const {
    // Make temporary wrap objects.
    cv::Mat mat_map_x( out_h, out_w, CV_32FC1, map_x );
    cv::Mat mat_map_y( out_h, out_w, CV_32FC1, map_y );
    cv::Mat mat_cross( cross_height, cross_width, CV_32FC1, image_cross );
    cv::Mat mat_out( out_h, out_w, CV_32FC1, out );

    // Sample.
    cv::remap( mat_cross, mat_out, mat_map_x, mat_map_y, cv::INTER_NEAREST );
}