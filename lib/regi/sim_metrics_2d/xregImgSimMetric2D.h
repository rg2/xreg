/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef XREGIMGSIMMETRIC2D_H_
#define XREGIMGSIMMETRIC2D_H_

#include "xregCommon.h"

namespace xreg
{

// Forward Declarations
class  RayCaster;
struct H5ReadWriteInterface;

/// \brief Base class for computing a similarity metric between a fixed (static)
///        image and a collection of moving (changing) images.
///
/// Currently, this assumes 2D images and is designed primarily to be used in an
/// intensity-based 2D/3D registration algorithm.
class ImgSimMetric2D
{
public:
  using Scalar       = RayCastPixelScalar;
  using Image        = itk::Image<Scalar,2>;
  using ImagePtr     = Image::Pointer;
  using ScalarList   = std::vector<Scalar>;
  using MaskScalar   = unsigned char;
  using ImageMask    = itk::Image<MaskScalar,2>;
  using ImageMaskPtr = ImageMask::Pointer;

  class UnsupportedOperationException { };

  /// \brief Constructor - trivial does not perform any work.
  ImgSimMetric2D() = default;

  virtual ~ImgSimMetric2D() { }
  
  // No copying
  ImgSimMetric2D(const ImgSimMetric2D&) = delete;
  ImgSimMetric2D& operator=(const ImgSimMetric2D&) = delete;

  /// \brief Sets the fixed image to be used.
  ///
  /// THIS IMAGE MAY BE MODIFIED BY THE SIMILARITY METRIC COMPUTATION. For example,
  /// to compute a zero-mean version without allocating additional storage.
  /// This needs to be set prior to calling allocate_resources()
  void set_fixed_image(ImagePtr fixed_img);

  /// \brief Returns the fixed image used during computation
  ImagePtr fixed_image();

  /// \brief Allocates resources that will be used for any subsequent similarity
  ///        value computations.
  ///
  /// In this class, this call initializes the buffer to store similarity values
  /// for each similarity metric evaluation.
  /// Child classes should call the parent's allocate_resources() at the beginning
  /// of their implementation of allocate_resources().
  virtual void allocate_resources();

  /// \brief Perform the similarity computations between each moving image and
  ///        the fixed image.
  ///
  /// allocate_resources() must be prior to this.
  /// Must be implemented by the child class.
  virtual void compute() = 0;

  /// \brief Sets the number of moving images (the number of similarity metric computations)
  ///
  /// This needs to be set prior to calling allocate_resources()
  void set_num_moving_images(const size_type n);

  /// \brief Retrieve the number of moving images (the number of similarity metric computations)
  size_type num_moving_images() const;

  /// \brief Retrieve a reference to a similarity value.
  ///
  /// Should only be called after calling allocate_resources()
  Scalar& sim_val(const size_type mov_img_idx);

  /// \brief Retrieve a const reference to a similarity value.
  ///
  /// Should only be called after calling allocate_resources()
  const Scalar& sim_val(const size_type mov_img_idx) const;

  /// \brief Retrieves a reference to the list of similarity values.
  ScalarList& sim_vals();

  /// \brief Retrieves a const reference to the list of similarity values.
  const ScalarList& sim_vals() const;

  /// \brief Sets the moving images buffer to use from a ray caster.
  ///
  /// This should always be called after calling allocate_resources() on the
  /// ray caster object and before calling allocate_resources() on the
  /// similarity object.
  virtual void set_mov_imgs_buf_from_ray_caster(RayCaster* ray_caster, const size_type proj_offset = 0);

  /// \brief Sets a host buffer to retrieve moving images from.
  virtual void set_mov_imgs_host_buf(Scalar* mov_imgs_buf, const size_type proj_offset = 0) = 0;

  /// \brief Sets a mask that should be applied to fixed and moving images.
  ///
  /// It is up to the child classes to apply the mask at the appropriate time.
  /// For example, NCC and SSD will apply the mask directly on the fixed and
  /// moving images, whereas Grad-NCC and Grad-Diff should apply the mask on
  /// the gradient images.
  void set_mask(ImageMaskPtr mask);

  ImageMaskPtr mask();

  void set_save_aux_info(const bool save_aux);

  virtual std::shared_ptr<H5ReadWriteInterface> aux_info();

protected:
  size_type num_pix_per_proj();

  void process_updated_mask();
  
  virtual void process_mask();

  ImagePtr fixed_img_;

  size_type num_mov_imgs_ = 0;

  ScalarList sim_vals_;

  ImageMaskPtr mask_;

  bool mask_updated_ = true;

  bool save_aux_info_ = false;
};

}  // xreg

#endif

