// Generated by gencpp from file fisheye_camera_matrix/perspective_projection.msg
// DO NOT EDIT!


#ifndef FISHEYE_CAMERA_MATRIX_MESSAGE_PERSPECTIVE_PROJECTION_H
#define FISHEYE_CAMERA_MATRIX_MESSAGE_PERSPECTIVE_PROJECTION_H

#include <ros/service_traits.h>


#include <fisheye_camera_matrix/perspective_projectionRequest.h>
#include <fisheye_camera_matrix/perspective_projectionResponse.h>


namespace fisheye_camera_matrix
{

struct perspective_projection
{

typedef perspective_projectionRequest Request;
typedef perspective_projectionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct perspective_projection
} // namespace fisheye_camera_matrix


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::fisheye_camera_matrix::perspective_projection > {
  static const char* value()
  {
    return "fac65d0011a9cb3cb4a513954c0b38a5";
  }

  static const char* value(const ::fisheye_camera_matrix::perspective_projection&) { return value(); }
};

template<>
struct DataType< ::fisheye_camera_matrix::perspective_projection > {
  static const char* value()
  {
    return "fisheye_camera_matrix/perspective_projection";
  }

  static const char* value(const ::fisheye_camera_matrix::perspective_projection&) { return value(); }
};


// service_traits::MD5Sum< ::fisheye_camera_matrix::perspective_projectionRequest> should match 
// service_traits::MD5Sum< ::fisheye_camera_matrix::perspective_projection > 
template<>
struct MD5Sum< ::fisheye_camera_matrix::perspective_projectionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::fisheye_camera_matrix::perspective_projection >::value();
  }
  static const char* value(const ::fisheye_camera_matrix::perspective_projectionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::fisheye_camera_matrix::perspective_projectionRequest> should match 
// service_traits::DataType< ::fisheye_camera_matrix::perspective_projection > 
template<>
struct DataType< ::fisheye_camera_matrix::perspective_projectionRequest>
{
  static const char* value()
  {
    return DataType< ::fisheye_camera_matrix::perspective_projection >::value();
  }
  static const char* value(const ::fisheye_camera_matrix::perspective_projectionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::fisheye_camera_matrix::perspective_projectionResponse> should match 
// service_traits::MD5Sum< ::fisheye_camera_matrix::perspective_projection > 
template<>
struct MD5Sum< ::fisheye_camera_matrix::perspective_projectionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::fisheye_camera_matrix::perspective_projection >::value();
  }
  static const char* value(const ::fisheye_camera_matrix::perspective_projectionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::fisheye_camera_matrix::perspective_projectionResponse> should match 
// service_traits::DataType< ::fisheye_camera_matrix::perspective_projection > 
template<>
struct DataType< ::fisheye_camera_matrix::perspective_projectionResponse>
{
  static const char* value()
  {
    return DataType< ::fisheye_camera_matrix::perspective_projection >::value();
  }
  static const char* value(const ::fisheye_camera_matrix::perspective_projectionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FISHEYE_CAMERA_MATRIX_MESSAGE_PERSPECTIVE_PROJECTION_H
