#pragma once
#ifndef PCL_WRAPPER_TYPES_H
#define PCL_WRAPPER_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#pragma warning(disable: 4819)

struct EIGEN_ALIGN16 _PclRichPoint
{
  EIGEN_ALIGN16 union
  {
    float data[4];
    struct
    {
      float x;
      float y;
      float z;
      union
      {
        struct
        {
          uint8_t b;
          uint8_t g;
          uint8_t r;
          uint8_t a;
        };
        float rgb;
      };
    };
  };

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); }
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); }
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); }
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); }
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); }
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); }
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

  inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); }
  inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); }
  inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, a)); }
  inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, a)); }

  EIGEN_ALIGN16 union
  {
    float data_n[4];
    float normal[3];
    struct
    {
      float normal_x;
      float normal_y;
      float normal_z;
      union
      {
        struct
        {
          uint8_t e1;
          uint8_t e2;
          uint8_t e3;
          uint8_t label;
        };
        float eigen;
      };
    };
  };

  EIGEN_ALIGN16 union
  {
    float data_o[4];
    float orientation[3];
    struct
    {
      float orientation_x;
      float orientation_y;
      float orientation_z;
      float probability;
    };
  };

  inline Eigen::Map<Eigen::Vector3f> getNormalVector3fMap () { return (Eigen::Vector3f::Map (data_n)); }
  inline const Eigen::Map<const Eigen::Vector3f> getNormalVector3fMap () const { return (Eigen::Vector3f::Map (data_n)); }
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getNormalVector4fMap () { return (Eigen::Vector4f::MapAligned (data_n)); }
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getNormalVector4fMap () const { return (Eigen::Vector4f::MapAligned (data_n)); }

  EIGEN_ALIGN16 union
  {
    float data_user[4];
    struct
    {
      union
      {
        float curvature;
        float flatness;
      };
      union
      {
        float user;
        float thickness;
      };
      int segment_id;
      int organ_id;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PclRichPoint : public _PclRichPoint
{
  static const int ID_UNINITIALIZED = -1;
  static const uint8_t LABEL_UNINITIALIZED = 0;
  static const uint8_t LABEL_POT =      1;
  static const uint8_t LABEL_NOISE =    2;
  static const uint8_t LABEL_PLANT =    3;

  static const uint8_t LABEL_LEAF =     16;
  static const uint8_t LABEL_FLOWER =   17;

  static const uint8_t LABEL_STEM =     32;
  static const uint8_t LABEL_PETIOLE =  33;
  static const uint8_t LABEL_MERISTEM = 34;
  static const uint8_t LABEL_TRUNK =    35;

  inline PclRichPoint (const _PclRichPoint &p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    rgb = p.rgb;

    normal_x = p.normal_x;
    normal_y = p.normal_y;
    normal_z = p.normal_z;
    eigen = p.eigen;

    orientation_x = p.orientation_x;
    orientation_y = p.orientation_y;
    orientation_z = p.orientation_z;
    probability = p.probability;

    curvature = p.curvature;
    user = p.user;
    segment_id = p.segment_id;
    organ_id = p.organ_id;
  }

  inline PclRichPoint (float _x, float _y, float _z)
  {
    x = _x; y = _y; z = _z;
    r = g = b = 0;
    a = 255;

    normal_x = normal_y = normal_z = 0.0f;
    e1 = e2 = e3 = 0;
    label = LABEL_UNINITIALIZED;

    orientation_x = orientation_y = orientation_z = 0.0f;
    probability = 0.0f;

    curvature = user = 0.0f;
    segment_id = organ_id = ID_UNINITIALIZED;
  }

  inline PclRichPoint ()
  {
    x = y = z = 0.0f;
    r = g = b = 0;
    a = 255;

    normal_x = normal_y = normal_z = 0.0f;
    e1 = e2 = e3 = 0;
    label = LABEL_UNINITIALIZED;

    orientation_x = orientation_y = orientation_z = 0.0f;
    probability = 0.0f;

    curvature = user = 0.0f;
    segment_id = organ_id = ID_UNINITIALIZED;
  }

  template <class T>
  explicit inline PclRichPoint(const T& t)
  {
    x = t.x();
    y = t.y();
    z = t.z();
    r = g = b = 0;
    a = 255;

    normal_x = normal_y = normal_z = 0.0f;
    e1 = e2 = e3 = 0;
    label = LABEL_UNINITIALIZED;

    orientation_x = orientation_y = orientation_z = 0.0f;
    probability = 0.0f;

    curvature = user = 0.0f;
    segment_id = organ_id = ID_UNINITIALIZED;
  }

  template <class T>
  inline T cast(void) const
  {
    return T(x, y, z);
  }

  template <class T>
  inline T castNormal(void) const
  {
    return T(normal_x, normal_y, normal_z);
  }

  template <class T>
  inline T castOrientation(void) const
  {
    return T(normal_x, normal_y, normal_z);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator << (std::ostream& os, const PclRichPoint& p)
{
  os << "("
    << p.x << "," << p.y << "," << p.z
    << " - " << p.r << "," << p.g << "," << p.b << "," << p.a
    << " - " << p.normal_x << "," << p.normal_y << "," << p.normal_z
    << " - " << p.e1 << "," << p.e2 << "," << p.e3
    << " - " << p.orientation_x << "," << p.orientation_y << "," << p.orientation_z
    << " - " << p.probability
    << " - " << p.label
    << " - " << p.curvature
    << " - " << p.user
    << " - " << p.segment_id
    << " - " << p.organ_id
    << ")";

  return (os);
}

POINT_CLOUD_REGISTER_POINT_WRAPPER(PclRichPoint, _PclRichPoint)
POINT_CLOUD_REGISTER_POINT_STRUCT(_PclRichPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, rgb, rgb)

  (float, normal_x, normal_x)
  (float, normal_y, normal_y)
  (float, normal_z, normal_z)
  (float, eigen, eigen)

  (float, orientation_x, orientation_x)
  (float, orientation_y, orientation_y)
  (float, orientation_z, orientation_z)
  (float, probability, probability)

  (float, curvature, curvature)
  (float, user, user)
  (int, segment_id, segment_id)
  (int, organ_id, organ_id)
)

typedef pcl::PointCloud<PclRichPoint> PclRichPointCloud;

struct _PclPoint
{
  EIGEN_ALIGN16 union
  {
    float data[4];
    struct
    {
      float x;
      float y;
      float z;
    };
  };

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); }
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); }
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); }
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); }
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); }
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); }
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PclPoint : public _PclPoint
{
  inline PclPoint (const _PclPoint &p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
  }

  inline PclPoint ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
  }

  inline PclPoint (float _x, float _y, float _z)
  {
    x = _x; y = _y; z = _z;
    data[3] = 1.0f;
  }

  template <class T>
  explicit inline PclPoint(const T& t)
  {
    x = t.x();
    y = t.y();
    z = t.z();
    data[3] = 1.0f;
  }

  template <class T>
  inline T cast(void) const
  {
    return T(x, y, z);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator << (std::ostream& os, const PclPoint& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << ")";
  return (os);
}

POINT_CLOUD_REGISTER_POINT_WRAPPER(PclPoint, _PclPoint)
POINT_CLOUD_REGISTER_POINT_STRUCT(_PclPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  )

typedef pcl::PointCloud<PclPoint> PclPointCloud;

template <class Matrix>
class PclMatrixCaster {
public:
  PclMatrixCaster(const Eigen::Matrix4f& m)
    : m_(m)
  {}

  PclMatrixCaster(const Matrix& m)
  {
    for (int i = 0; i < 4; ++ i)
      for (int j = 0; j < 4; ++ j)
        m_(i, j) = m(j, i);
  }

  operator Eigen::Matrix4f() const
  {
    return m_;
  }

  operator Matrix() const
  {
    Matrix m;
    for (int i = 0; i < 4; ++ i)
      for (int j = 0; j < 4; ++ j)
        m(i, j) = m_(j, i);
    return m;
  }

private:
  Eigen::Matrix4f m_;
};

struct NullDeleter {void operator()(void const *) const {}};

#endif // PCL_WRAPPER_TYPES_H