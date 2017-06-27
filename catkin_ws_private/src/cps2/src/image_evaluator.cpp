#include <math.h>
#include <opencv2/imgproc.hpp>

#ifdef DEBUG_IE
#include <stdio.h>
#include <opencv2/highgui.hpp>
#endif

#include "image_evaluator.hpp"

namespace cps2 {

#ifdef DEBUG_IE
int d_img_width  = 360;
int d_img_height = 240;
cv::Size d_win_size(d_img_width, d_img_height);
cv::Mat d_win_img(d_img_height, 2 * d_img_width + 20, CV_8UC1);
#endif

void ImageEvaluator::generateKernel() {
  int center = kernel_size / 2;
  float acc  = 0;

  cv::Mat kernel(kernel_size, kernel_size, CV_32FC1);

  for(int r = 0; r < kernel_size; ++r) {
    int y = r - center;

    for(int c = 0; c < kernel_size; ++c) {
      int x = c - center;
      kernel.at<float>(r, c) = exp( (-x*x - y*y) / (2 * kernel_stddev*kernel_stddev) );
      acc += kernel.at<float>(r, c);
    }
  }

  this->kernel = kernel;
}

int ImageEvaluator::applyKernel(const cv::Mat &img, int x, int y) {
  int ks2     = kernel_size / 2;
  int x_lb    = std::max(0, x - ks2);
  int x_ub    = std::min(img.cols, x + ks2 + 1);
  int y_lb    = std::max(0, y - ks2);
  int y_ub    = std::min(img.rows, y + ks2 + 1);
  float acc_k = 0;
  float acc_v = 0;

  for(int r = y_lb; r < y_ub; ++r) {
    int ky = r - y + ks2;

    for(int c = x_lb; c < x_ub; ++c) {
      int kx = c - x + ks2;

      acc_k += kernel.at<float>(ky, kx);
      acc_v += kernel.at<float>(ky, kx) * img.at<uchar>(r, c);
    }
  }

  return (int)(acc_v / acc_k);
}

ImageEvaluator::ImageEvaluator(int _mode, int _resize_scale, int _kernel_size, float _kernel_stddev) :
    mode(_mode),
    resize_scale(_resize_scale),
    kernel_stddev(_kernel_stddev)
{
  kernel_size = 2 * (_kernel_size / 2) + 1;

  generateKernel();
}

ImageEvaluator::~ImageEvaluator() {

}

cv::Mat ImageEvaluator::transform(const cv::Mat &img, const cv::Point3f &pos_image) {
  int dim_y = img.rows / resize_scale;
  int dim_x = img.cols / resize_scale;
  int cx    = dim_x / 2;
  int cy    = dim_y / 2;

  cv::Mat img_tf(dim_y, dim_x, CV_8UC1);

  for(int c = 0; c < dim_x; ++c) {
    int sx = c - cx;

    for(int r = 0; r < dim_y; ++r) {
      int sy  = r - cy;
      float x = resize_scale * (sx * cosf(pos_image.z) - sy * sinf(pos_image.z) ) + pos_image.x;
      float y = resize_scale * (sx * sinf(pos_image.z) + sy * cosf(pos_image.z) ) + pos_image.y;

      if(x >= 0 && y >= 0 && x < img.cols && y < img.rows)
        img_tf.at<uchar>(r, c) = applyKernel(img, (int)x, (int)y);
      else
        img_tf.at<uchar>(r, c) = 0;
    }
  }

  return img_tf;
}

float ImageEvaluator::evaluate(cv::Mat &img1, cv::Mat &img2) {
  float error_pixels = 0;

#ifdef DEBUG_IE
  int mode_backup = mode;
  mode = IE_MODE_PIXELS;
#endif

  if(mode == IE_MODE_PIXELS) {
    int pixels = img1.rows * img1.cols;

    for(int r = 0; r < img1.rows; ++r)
      for(int c = 0; c < img1.cols; ++c)
        if(img1.at<uchar>(r, c) == 0 || img2.at<uchar>(r, c) == 0)
          --pixels;
        else
          error_pixels += fabs( (float)img1.at<uchar>(r, c) - img2.at<uchar>(r, c) );

    if(pixels == 0)
      error_pixels = 1;
    else
      error_pixels /= 255 * pixels;
  }

  float error_centroids = 0;

#ifdef DEBUG_IE
  mode = IE_MODE_CENTROIDS;
#endif

  if(mode == IE_MODE_CENTROIDS) {
    float map_m00 = 0;
    float map_m10 = 0;
    float map_m01 = 0;
    float img_m00 = 0;
    float img_m10 = 0;
    float img_m01 = 0;

    for(int r = 0; r < img1.rows; ++r)
      for(int c = 0; c < img1.cols; ++c) {
        if(img1.at<uchar>(r, c) != 0) {
          map_m00 += img1.at<uchar>(r, c);
          map_m10 += img1.at<uchar>(r, c) * c;
          map_m01 += img1.at<uchar>(r, c) * r;
        }

        if(img2.at<uchar>(r, c) != 0) {
          img_m00 += img2.at<uchar>(r, c);
          img_m10 += img2.at<uchar>(r, c) * c;
          img_m01 += img2.at<uchar>(r, c) * r;
        }
      }

    error_centroids = fabs(map_m10 / map_m00 - img_m10 / img_m00) / img1.cols
      + fabs(map_m01 / map_m00 - img_m01 / img_m00) / img1.rows;
  }

#ifdef DEBUG_IE
  cv::resize(img2, d_win_img(cv::Rect(0 ,0, 360, 240) ), d_win_size, 0, 0, cv::INTER_NEAREST);
  cv::resize(img1, d_win_img(cv::Rect(380 ,0, 360, 240) ), d_win_size, 0, 0, cv::INTER_NEAREST);
  cv::imshow("test", d_win_img);

  mode = mode_backup;

  printf("\n========== error: ==========\n");
  printf("pixelwise: %f\n", error_pixels);
  printf("centroids: %f\n", error_centroids);
#endif

  if(mode == IE_MODE_CENTROIDS)
    return error_centroids;

  return error_pixels;
}

} /* namespace cps2 */
