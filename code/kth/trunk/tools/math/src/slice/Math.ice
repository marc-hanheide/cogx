#ifndef MATH_ICE
#define MATH_ICE

module cogx {

module Math {

  struct Vector2 {
    double x;
    double y;
  };

  struct Vector3 {
    double x;
    double y;
    double z;
  };

  struct Matrix33 {
    double m00;
    double m01;
    double m02;

    double m10;
    double m11;
    double m12;

    double m20;
    double m21;
    double m22;
  };

  struct Rect2 {
    Vector2 pos;
    double width;
    double height;
  };

  struct Sphere3 {
    Vector3 pos;
    double rad;
  };

  // 3D pose consisting of position vector and rotation matrix
  struct Pose3 {
    Vector3 pos;
    Matrix33 rot;
  };
};
};

#endif

