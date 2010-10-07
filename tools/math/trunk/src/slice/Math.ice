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

  /**
   * 2D axis-parallel rectangle.
   * Position refers to the position of the center. So the rectangle spans the
   * space [pos.x - width/2, pos.x + width/2] etc.
   */
  struct Rect2 {
    Vector2 pos;  
    // position of the center
    double width;
    double height;
  };

  struct Sphere3 {
    Vector3 pos;
    double rad;
  };

  /**
   * 3D axis-parallel box.
   * Position refers to the position of the center. So the box spans the space
   * [pos.x - size.x/2, pos.x + size.x/2] etc.
   */
  struct Box3 {
    Vector3 pos;  
     // position of the center
    Vector3 size;  
    // sizes in x, y and z direction
  };

  // 3D pose consisting of position vector and rotation matrix
  struct Pose3 {
    Vector3 pos;
    Matrix33 rot;
  };
  
  struct Plane3 {
  	double a;
  	double b;
  	double c;
  	double d;
  };
  
};
};

#endif

