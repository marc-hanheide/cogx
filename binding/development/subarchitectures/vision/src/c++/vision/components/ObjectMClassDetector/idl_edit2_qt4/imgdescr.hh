#ifndef IMGDESCR_HH
#define IMGDESCR_HH
#include <vector>
#include <string>

using namespace std;

struct Rect{
  int x1;
  int y1;
  int x2;
  int y2;
  double score;
};

class ImgDescr {
public:
  string name;
  vector<Rect> RectList;
};

#endif
