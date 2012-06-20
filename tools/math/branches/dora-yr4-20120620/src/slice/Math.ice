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
    // position of the center
    Vector2 pos;  
    double width;
    double height;
  };

  struct Sphere3 {
    // position of the center
    Vector3 pos;
    double rad;
  };

  /**
   * 3D axis-parallel box.
   * Position refers to the position of the center. So the box spans the space
   * [pos.x - size.x/2, pos.x + size.x/2] etc.
   */
  struct Box3 {
    // position of the center
    Vector3 pos;  
    // sizes in x, y and z direction
    Vector3 size;  
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

  /**
   * RGB color
   * NOTE: bytes in ICE are -128..127! So you will need to cast to an unsigned
   * char in your code.
   * @author Michael Zillich
   */
  struct ColorRGB {
    byte r;
    byte g;
    byte b;
  };
  
};
};

#endif

