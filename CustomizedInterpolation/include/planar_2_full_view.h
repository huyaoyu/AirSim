//
// Created by yaoyu on 5/22/21.
//

#ifndef PLANAR2FULLVIEW_PLANAR_2_FULL_VIEW_H
#define PLANAR2FULLVIEW_PLANAR_2_FULL_VIEW_H

// Standard headers.
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace p2f {
    // The face names used by the right-hand sphere frame.
    // Different from the left-hand coordinate frame used in Unreal Engine.
    enum class FaceName {
        POS_X = 0,
        NEG_X,
        POS_Y,
        NEG_Y,
        POS_Z,
        NEG_Z
    };

    /**
     * Convert the Unreal Engine face index to a \ref FaceName enum.
     * @param ue_face The Unreal Engine face index.
     * @return The associated \ref FaceName enum.
     */
    FaceName unreal_engine_face_2_face_name(int ue_face);

    static inline std::pair<int, int> ue_pos_x_2_ocv(int width, int x, int y) {
        return { width - 1 - y, x };
    }

    static inline std::pair<int, int> ue_neg_x_2_ocv(int width, int x, int y) {
        return { y, width - 1 - x };
    }

    static inline std::pair<int, int> ue_pos_y_2_ocv(int width, int x, int y) {
        return { width - 1 - x, width - 1 - y };
    }

    static inline std::pair<int, int> ue_neg_y_2_ocv(int width, int x, int y) {
        return { x, y };
    }

    static inline std::pair<int, int> ue_pos_z_2_ocv(int width, int x, int y) {
        return { width - 1 - y, x };
    }

    static inline std::pair<int, int> ue_neg_z_2_ocv(int width, int x, int y) {
        return { width - 1 - y , x };
    }

    extern const std::vector<std::function<std::pair<int, int>(int, int, int)>> ue_coor_2_ocv;

    /**
     * Merge 6 pin-hole images into a full-view panorama using nearest neighbor interpolation.
     */
    class Planar2FullView{
    public:
        /**
         * Constructor.
         * @param width The width of the input image. Assuming height = width for the input.
         * @param out_h The height of the output full-view image.
         * @param out_w The width of the output full-view image.
         */
        Planar2FullView(int width, int out_h, int out_w);
        ~Planar2FullView();

        int get_height() const { return height; }
        int get_width() const { return width; }
        /**
         * Return the height of the image cross saved internally.
         * @return Height of the image cross.
         */
        int get_cross_height() const { return cross_height; }
        /**
         * Return the width of the image cross saved internally.
         * @return Width of the image cross.
         */
        int get_cross_width() const { return cross_width; }

        /**
         * Return the pixel numbers of the internal image cross.
         * @return Pixel number of the internal image cross.
         */
        std::size_t get_cross_size() const { return cross_height * cross_width; }
        /**
         * Return the total bytes used for saving the internal image cross.
         * @return Bytes of memory of the internal image cross.
         */
        std::size_t get_cross_memory_size() const { return cross_height * cross_width * sizeof(float); }

        /**
         * Convert a single face image and copy the converted content into the internal image cross.
         * @tparam PixelT The type of the input pixel.
         * @param face The face name.
         * @param pixels The input pixels.
         * @param converter A callable that does single value conversion.
         */
        template < typename PixelT >
        void convert_and_copy(
                const FaceName& face,
                const PixelT* pixels,
                const std::function<float(const PixelT&)>& converter,
                int stride );

        template < typename PixelT >
        void convert_and_copy(
                const FaceName& face,
                const PixelT* pixels,
                const std::function<float(const PixelT&)>& converter,
                const std::function<std::pair<int, int>(int, int, int)>& coor_map,
                int stride);

        /**
         * Create padding for the internal image cross. Should be called once all faces are copied to the image cross.
         */
        void make_padding();

        /**
         * Write the internal image cross to a file. A visualization version of the floating point image cross is
         * created and witten to the target file. The floating point image is normalized and clipped by \p min_val
         * and \p max_val.
         * @param fn The target filename.
         * @param min_val Mininum value for normalization and clipping.
         * @param max_val Maximum value for normalization and clipping.
         * @return true for success.
         */
        bool write_image_cross_vis(const std::string& fn, float min_val, float max_val) const;

        /**
         * Generate the full-view panorama image by sampling. The caller is responsible to allocate the memory for
         * \p out.
         * @param out The full-view panorama image.
         */
        void sample( float* out ) const;

    private:
        /**
         * Create the mapping coordinates.
         */
        void create_map_xy();

        /**
         * Return the pixel index of a single image face in the internal image cross.
         * @param face The face name.
         * @return x, y pixel coordinates.
         */
        std::pair<int, int> get_face_start_index(const FaceName& face) const;

        /**
         * Copy from an array to another using customizable strides.
         * @param from The source array.
         * @param stride_from The stride for the source.
         * @param to The destination array.
         * @param stride_to The stride for the destination.
         * @param n Number of elements to copy.
         */
        static void copy_by_stride(
                const float* from, int stride_from,
                float* to, int stride_to,
                int n);

    private:
        const int H_PAD = 1; /// Single-side vertical padding. Total padding will be 2*H_PAD.
        const int W_PAD = 1; /// single-side horizontal padding. Total padding will be 2*W_PAD.

        int height; /// Height of the input face. Equals to width.
        int width; /// Width of the input face.
        int cross_height; /// Height of the internal image cross.
        int cross_width; /// Width of the internal image cross.

        int out_h; /// Height of the output full-view panorama.
        int out_w; /// Width of the output full-view panorama.

        float* image_cross; /// The internal image cross.
        float* map_x; /// The x-coordinates of the mapping.
        float* map_y; /// The y-coordinates of the mapping.
    };

    template < typename PixelT >
    void Planar2FullView::convert_and_copy(
            const FaceName& face,
            const PixelT* pixels,
            const std::function<float(const PixelT&)>& converter,
            int stride ) {
        // Get the starting index of the face.
        const std::pair<int, int> indices = get_face_start_index(face);
        const int idx_h = indices.first;
        const int idx_w = indices.second;

        for ( int i = 0; i < height; i++ ) {
            const std::size_t h = idx_h + i;
            const int pixel_h_offset = i * width * stride;
            for ( int j = 0; j < width; j++ ) {
                const std::size_t idx_ic = h * cross_width + idx_w + j; // Image cross index.
                const std::size_t idx_pixel = pixel_h_offset + j * stride; // Source index.
                image_cross[idx_ic] = converter( pixels[idx_pixel] );
            }
        }
    }

    template < typename PixelT >
    void Planar2FullView::convert_and_copy(
            const FaceName& face,
            const PixelT* pixels,
            const std::function<float(const PixelT&)>& converter,
            const std::function<std::pair<int, int>(int, int, int)>& coor_map,
            int stride) {
        // Get the starting index of the face in the image cross.
        const std::pair<int, int> indices = get_face_start_index(face);
        const int idx_h = indices.first;
        const int idx_w = indices.second;

        const int total_pixel = height * width; // Max 2,147,483,647.

        // Loop over all the in coming pixels.
        for ( int i = 0; i < total_pixel; i++ ) {
            // Coordinate in the input pixel space.
            const int py = i / width;
            const int px = i % width;

            // Coordinate in the target patch of the image cross.
            std::pair<int, int> target_coor = coor_map(width, px, py);
            const int idx_target =
                      (idx_h + target_coor.second) * cross_width
                    + idx_w + target_coor.first;
            // Memory location in the incoming pixels.
            const int idx_pixel = i * stride;

            image_cross[idx_target] = converter( pixels[idx_pixel] );
        }
    }
} // namespace p2f

#endif //PLANAR2FULLVIEW_PLANAR_2_FULL_VIEW_H
