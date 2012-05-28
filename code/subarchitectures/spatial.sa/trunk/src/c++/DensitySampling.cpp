#include <vector>
#include "DensitySampling.hpp"
#include <float.h>
#include <Navigation/LocalGridMap.hh>
#include <cogxmath.h>
#include <Pose3.h>
#include <fstream>
#include "SpatialGridMap.hh"
#include "KDEFunctors.hh"
#include <iostream>
using namespace std;
using namespace cogx::Math;

// Represents an axis-aligned equidistant-sampled box shape, whose position 
// is relative to the position (but not the orientation) of a base object
// (object1). The points represent samples of the distribution of a specific
// spatial relation between object1 and another object, object2.
// For each spatial sample point are maintained the values of the relation,
// if object2 were in that position, and there is one value for each of a
// finite set of orientations of object1 and object2.

// Exactly which orientations can be specified externally. For example,
// if the pose of object1 is actually known, there needs be only one.

// Two clouds A and B can be composited together; this presumes that A.object2 
// and B.object1 are the same object, and that the set of
// orientations of A.object2 and B.object1 are also the same.
// Chaining the cloud gives rise to a new cloud C, with
// C.object1 = A.object1
// C.orientations1 = A.orientations1
// C.object2 = B.object2
// C.orientations2 = B.orientations2
// C.samplingInterval = smallest common prime factor of A.samplingInterval and
//	B.samplingInterval
// C.sampleOffset = A.sampleOffset + B.sampleOffset
// 
// The values of each sample point of C are acquired by "convolving" 
// A and B; i.e., B is placed in every point in A and each of B's points weighted
// by the A point is put into C.

// Finally, a cloud can be KDE-sampled into a grid. This grid should have a cell size
// such that the sampling interval of the cloud is an integer multiple of it, or 
// aliasing effects will distort the result (more than necessary).

namespace spatial {

spatial::Object *
copyObject(const spatial::Object *o) {
  spatial::Object *ret;
  switch (o->type) {
  case OBJECT_HOLLOW_BOX: {
    HollowBoxObject *o1hb = (HollowBoxObject *) o;
    HollowBoxObject *hb = new HollowBoxObject;
    ret = hb;
    hb->thickness = o1hb->thickness;
    hb->sideOpen = o1hb->sideOpen;
    hb->radius1 = o1hb->radius1;
    hb->radius2 = o1hb->radius2;
    hb->radius3 = o1hb->radius3;
  }
    break;
  case OBJECT_BOX: {
    BoxObject *o1b = (BoxObject *) o;
    BoxObject *b = new BoxObject;
    ret = b;
    b->radius1 = o1b->radius1;
    b->radius2 = o1b->radius2;
    b->radius3 = o1b->radius3;
  }
    break;
  case OBJECT_PLANE: {
    PlaneObject *o1p = (PlaneObject *) o;
    PlaneObject *p = new PlaneObject;
    ret = p;
    p->shape = o1p->shape;
    p->radius1 = o1p->radius1;
    p->radius2 = o1p->radius2;
  }
    break;
  default:
    cerr << "Error: unsupported object type!\n";
    exit(1);
  }
  ret->type = o->type;
  ret->pose = o->pose;
  return ret;
}

SampleCloud::SampleCloud(const spatial::Object *o1, const spatial::Object *o2,
    spatial::SpatialRelationType _rel, Vector3 offset, double intervalQuantum,
    int intervalMultiplier, int xExt, int yExt, int zExt,
    const vector<Matrix33> &orientations1,
    const vector<Matrix33> &orientations2) :
  rel(_rel), sampleOffset(offset), sampleIntervalQuantum(intervalQuantum),
      kernelRadius(0.0), sampleIntervalMultiplier(intervalMultiplier), xExtent(
          xExt), yExtent(yExt), zExtent(zExt), object1Orientations(
          orientations1), object2Orientations(orientations2) {
  object1 = copyObject(o1);
  object2 = copyObject(o2);
}

SampleCloud::SampleCloud(const SampleCloud &a) :
  sampleOffset(a.sampleOffset), sampleIntervalQuantum(a.sampleIntervalQuantum),
      kernelRadius(a.kernelRadius), sampleIntervalMultiplier(
          a.sampleIntervalMultiplier), xExtent(a.xExtent), yExtent(a.yExtent),
      zExtent(a.zExtent), object1Orientations(a.object1Orientations),
      object2Orientations(a.object2Orientations) {
  object1 = copyObject(a.object1);
  object2 = copyObject(a.object2);
}

SampleCloud &
SampleCloud::operator=(const SampleCloud &a) {
  sampleOffset = a.sampleOffset;
  sampleIntervalQuantum = a.sampleIntervalQuantum;
  sampleIntervalMultiplier = a.sampleIntervalMultiplier;
  xExtent = a.xExtent;
  yExtent = a.yExtent;
  zExtent = a.zExtent;
  object1Orientations = a.object1Orientations;
  object2Orientations = a.object2Orientations;
  values = a.values;

  if (object1 != 0)
    delete object1;
  if (object2 != 0)
    delete object2;

  object1 = copyObject(a.object1);
  object2 = copyObject(a.object2);
  kernelRadius = a.kernelRadius;
  return *this;
}

inline bool isInTriangle(double x, double y, const vector<Vector3> &triangle) {
  for (int i = 0; i < 3; i++) {
    int iplus = i == 2 ? 0 : i + 1;
    Vector3 side = triangle[iplus] - triangle[i];
    Vector3 diff = vector3(x, y, 0) - triangle[i];
    if (cross(side, diff).z < 0)
      return false;
  }
  return true;
}

struct OffsetKernel {
  int minxi;
  int maxxi;
  int minyi;
  int maxyi;
  vector<double> weights;
};

void DensitySampler::sampleBinaryRelationRecursively(const vector<
    SpatialRelationType> &relations, const vector<spatial::Object *> &objects,
    int currentLevel, Cure::LocalGridMap<double> &outMap, double &total,
    const vector<Vector3> &triangle, double baseOnness) {
  spatial::Object *supportObject = objects[currentLevel + 1];
  if (supportObject->pose.pos.x == -FLT_MAX) {
    cerr << "Error! Support object pose uninitialized!\n";
    return;
  }

  SpatialRelationType relationType = relations[currentLevel];
  spatial::Object *onObject = objects[currentLevel];

  Pose3 oldPose = onObject->pose;

  double frameRadius;
  double maxVertical;
  double minVertical;
  double maxLateral;
  switch (relationType) {

  case RELATION_ON:
    if (supportObject->type == spatial::OBJECT_PLANE) {
      spatial::PlaneObject &table1 = (spatial::PlaneObject &) (*supportObject);
      if (table1.shape == spatial::PLANE_OBJECT_RECTANGLE) {
        frameRadius = table1.radius1 > table1.radius2 ? table1.radius1
            : table1.radius2;
      } else {
        cerr << "Unsupported object type!\n";
        return;
      }
    } else if (supportObject->type == spatial::OBJECT_BOX
        || supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
      spatial::BoxObject &box1 = (spatial::BoxObject &) (*supportObject);
      frameRadius = box1.radius1 > box1.radius2 ? box1.radius1 : box1.radius2;
      frameRadius = frameRadius > box1.radius3 ? frameRadius : box1.radius3;
    } else {
      cerr << "Unsupported object type!\n";
      return;
    }
    maxLateral = frameRadius * 1.5;
    minVertical = -frameRadius * 1.5;
    maxVertical = frameRadius * 3;
    cout << "maxLateral: " << maxLateral << "  minVertical" << minVertical
        << "  maxVertical: " << maxVertical << "\n";
    break;

  case RELATION_IN:
    if (supportObject->type == spatial::OBJECT_HOLLOW_BOX) {
      spatial::BoxObject &box1 = (spatial::BoxObject &) (*supportObject);
      frameRadius = box1.radius1 > box1.radius2 ? box1.radius1 : box1.radius2;
      frameRadius = frameRadius > box1.radius3 ? frameRadius : box1.radius3;
    } else {
      cerr << "Unsupported object type!\n";
      return;
    }
    maxLateral = frameRadius * 1.5;
    minVertical = -frameRadius * 1.5;

    maxVertical = frameRadius * 1.5;
    break;
  default:
    cerr << "Unsupported relation type!\n";
    exit(8);
  }

  int mapSize = outMap.getSize();
  double cellSize = outMap.getCellSize();

  double x1 = -maxLateral + supportObject->pose.pos.x;
  double x2 = maxLateral + supportObject->pose.pos.x;
  double y1 = -maxLateral + supportObject->pose.pos.y;
  double y2 = maxLateral + supportObject->pose.pos.y;
  double zmin = supportObject->pose.pos.z + minVertical;
  double zmax = supportObject->pose.pos.z + maxVertical;

  int sampleNumberTargetUsed;
  switch (objects.size() - 2 - currentLevel) {
  case 0: //Base level
    sampleNumberTargetUsed = m_sampleNumberTarget;
    break;
  case 1: //One level of indirection
  default:
    sampleNumberTargetUsed = m_sampleNumberTarget / 10;
    break;
  }

  // Distribute N samples over the whole volume
  double sampleVolume = (x2 - x1) * (y2 - y1) * (zmax - zmin);

  // samplingInterval: distance between successive samples.
  // Can be set to a non-multiple of cellSize, but that may lead to
  // aliasing problems.

  double samplingInterval = pow(sampleVolume / sampleNumberTargetUsed, 1 / 3.0);
  // Round samplingInterval upward to the nearest multiple of cellSize
  // (keeps the number of sampled positions less than sampleNumberTarget)
  samplingInterval = cellSize * (ceil(samplingInterval / cellSize));
  std::cout << "m_sampleNumberTarget: " << m_sampleNumberTarget << std::endl;
  std::cout << "sampleNumberTargetUsed: " << sampleNumberTargetUsed
      << std::endl;
  std::cout << "sampleVolume: " << sampleVolume << std::endl;
  std::cout << "samplingInterval: " << samplingInterval << std::endl;

  double kernelRadius = samplingInterval * m_kernelWidthFactor;
  std::cout << "kernelRadius: " << kernelRadius << std::endl;
  if (kernelRadius < cellSize)
    kernelRadius = cellSize;
  int kernelWidth = (int) (ceil(kernelRadius / cellSize) + 0.1);
  //How many kernelRadiuses per cell
  double kernelStep = cellSize / kernelRadius;

  vector<Matrix33> orientations;
  getRandomSampleSphere(orientations, m_orientationQuantization);

  double z0 = ((double) rand()) / RAND_MAX * samplingInterval;
  //Offset, 0-centered, along length samplingInterval
  double xOffset = (((double) rand()) / RAND_MAX) * samplingInterval;
  double yOffset = (((double) rand()) / RAND_MAX) * samplingInterval;
  for (double sx = xOffset + x1; sx < x2; sx += samplingInterval) {
    for (double sy = yOffset + y1; sy < y2; sy += samplingInterval) {
      if (triangle.size() > 0 && currentLevel == 0 && !isInTriangle(sx, sy,
          triangle))
        continue;

      //Put together a kernel offset by this much
      OffsetKernel kernel;

      int sampleMapX, sampleMapY;
      outMap.worldCoords2Index(sx, sy, sampleMapX, sampleMapY);

      // Offset of sample position from center of its containing cell
      double offsetInCellX = sx - cellSize * (floor(sx / cellSize) + 0.5);
      double offsetInCellY = sy - cellSize * (floor(sy / cellSize) + 0.5);

      kernel.minxi = -kernelWidth; //Relative indices of kernel patch extents
      kernel.minyi = -kernelWidth;
      kernel.maxxi = +kernelWidth;
      kernel.maxyi = +kernelWidth;

      double minx, miny;
      outMap.index2WorldCoords(kernel.minxi, kernel.minyi, minx, miny);
      minx -= offsetInCellX; //Relative coords of minxi, minyi in meters
      miny -= offsetInCellY;

      minx /= kernelRadius; //Relative coords of minxi, minyi in kernel radii
      miny /= kernelRadius;

      double sumWeights = 0.0;
      double x = minx;
      for (int xi = kernel.minxi; xi <= kernel.maxxi; xi++, x += kernelStep) {
        double y = miny;
        for (int yi = kernel.minyi; yi <= kernel.maxyi; yi++, y += kernelStep) {
          double sq = x * x + y * y;
          //double sqsum = 1 - sq;
          double sqsum = (1 - sq) * (1 - sq) * (1 - sq);
          if (sqsum > 0) {
            kernel.weights.push_back(sqsum);
            sumWeights += sqsum;
          } else {
            kernel.weights.push_back(0);
          }
        }
      }
      for (unsigned int i = 0; i < kernel.weights.size(); i++) {
        kernel.weights[i] = kernel.weights[i] / sumWeights;
      }

      double totalValueThisXY = 0.0; //Sum sample values of all samples at this
      //(x,y)
      for (double sz = z0 + zmin; sz < zmax; sz += samplingInterval) {
        double totalValueThisXYZ = 0;
        for (unsigned int orientationNo = 0; orientationNo
            < orientations.size(); orientationNo++) {
          onObject->pose.rot = orientations[orientationNo];
          onObject->pose.pos.x = sx;
          onObject->pose.pos.y = sy;
          onObject->pose.pos.z = sz;
          double value;
          if (relationType == RELATION_ON) {
            value = m_evaluator->evaluateOnness(supportObject, onObject);
          } else if (relationType == RELATION_IN) {
            value = m_evaluator->evaluateInness(supportObject, onObject);
          } else {
            cerr << "Error! Unknown binary relation type!\n";
            return;
          }

          if (currentLevel == 0) {
            totalValueThisXY += value;
            totalValueThisXYZ += value;
          } else {
            // Sample and recurse, if the value is above a threshold
            if (value > 0.5) {
              sampleBinaryRelationRecursively(relations, objects, currentLevel
                  - 1, outMap, total, triangle, value * baseOnness);
            }
          }
        }
      }

      if (currentLevel == 0) {
        // This is the trajector itself
        // Accumulate the kernel by this much
        int kernelCellNo = 0;
        for (int xi = sampleMapX + kernel.minxi; xi <= sampleMapX
            + kernel.maxxi; xi++) {
          for (int yi = sampleMapY + kernel.minyi; yi <= sampleMapY
              + kernel.maxyi; yi++, kernelCellNo++) {
            if (xi <= -mapSize || xi >= mapSize || yi <= -mapSize || yi
                >= mapSize)
              continue;

            outMap(xi, yi) += kernel.weights[kernelCellNo] * totalValueThisXY
                * baseOnness;
            total += kernel.weights[kernelCellNo] * totalValueThisXY
                * baseOnness;
          }
        }
      } else {
      }
    }
  }
  onObject->pose = oldPose;
}

SampleCloud *
DensitySampler::createRelativeSampleCloud(SpatialRelationType relationType,
    Object *o1, Object *o2, const vector<Matrix33> &orientations1,
    const vector<Matrix33> &orientations2, double cellSize) {
  // Find sample box extents depending on support object dimensions
  // and relation type
  double frameRadius;
  double maxVertical;
  double minVertical;
  double maxLateral;
  switch (relationType) {

  case RELATION_ON:
    if (o1->type == spatial::OBJECT_PLANE) {
      spatial::PlaneObject &table1 = (spatial::PlaneObject &) (*o1);
      if (table1.shape == spatial::PLANE_OBJECT_RECTANGLE) {
        frameRadius = table1.radius1 > table1.radius2 ? table1.radius1
            : table1.radius2;
      } else {
        cerr << "Unsupported object type!\n";
        return 0;
      }
    } else if (o1->type == spatial::OBJECT_BOX || o1->type
        == spatial::OBJECT_HOLLOW_BOX) {
      spatial::BoxObject &box1 = (spatial::BoxObject &) (*o1);
      frameRadius = box1.radius1 > box1.radius2 ? box1.radius1 : box1.radius2;
      frameRadius = frameRadius > box1.radius3 ? frameRadius : box1.radius3;
    } else {
      cerr << "Unsupported object type!\n";
      return 0;
    }
    maxLateral = frameRadius * 1.5;
    minVertical = -frameRadius * 1.5;
    maxVertical = frameRadius * 3;
    break;

  case RELATION_IN:
    if (o1->type == spatial::OBJECT_HOLLOW_BOX) {
      spatial::BoxObject &box1 = (spatial::BoxObject &) (*o1);
      frameRadius = box1.radius1 > box1.radius2 ? box1.radius1 : box1.radius2;
      frameRadius = frameRadius > box1.radius3 ? frameRadius : box1.radius3;
    } else {
      cerr << "Unsupported object type!\n";
      return 0;
    }
    maxLateral = frameRadius * 1.5;
    minVertical = -frameRadius * 1.5;

    maxVertical = frameRadius * 1.5;
    break;
  default:
    cerr << "Unsupported relation type!\n";
    exit(8);
  }

  double x1 = -maxLateral;
  double x2 = maxLateral;
  double y1 = -maxLateral;
  double y2 = maxLateral;
  double zmin = minVertical;
  double zmax = maxVertical;

  // Distribute N samples over the whole volume
  double sampleVolume = (x2 - x1) * (y2 - y1) * (zmax - zmin);

  // samplingInterval: distance between successive samples.
  // Can be set to a non-multiple of cellSize, but that may lead to
  // aliasing problems.

  double samplingInterval = pow(sampleVolume / m_sampleNumberTarget, 1 / 3.0);
  // Round samplingInterval upward to the nearest multiple of cellSize
  // (keeps the number of sampled positions less than sampleNumberTarget)
  samplingInterval = cellSize * (ceil(samplingInterval / cellSize));
  int samplingMultiplier = (int) (samplingInterval / cellSize + 0.1);

  double z0 = ((double) rand()) / RAND_MAX * samplingInterval;
  //Offset, 0-centered, along length samplingInterval
  double xOffset = (((double) rand()) / RAND_MAX - 0.5) * samplingInterval;
  double yOffset = (((double) rand()) / RAND_MAX - 0.5) * samplingInterval;

  int xExtent = (int) (maxLateral / samplingInterval);
  int yExtent = (int) (maxLateral / samplingInterval);
  int zExtent = (int) (maxVertical / samplingInterval);

  SampleCloud *ret = new SampleCloud(o1, o2, relationType, vector3(xOffset,
      yOffset, z0), cellSize, samplingMultiplier, xExtent, yExtent, zExtent,
      orientations1, orientations2);
  ret->kernelRadius = samplingInterval * m_kernelWidthFactor;

  return ret;
}

void DensitySampler::sampleBinaryRelationSystematically(const vector<
    SpatialRelationType> &relations, const vector<spatial::Object *> &objects,
    const std::vector<string> &objectLabels, double cellSize,
    SampleCloud &outCloud) {
  spatial::Object *supportObject = objects.back();
  bool randomizeBaseObjectPose = false;

  if (supportObject->pose.pos.x == -FLT_MAX) {
    randomizeBaseObjectPose = true;
    supportObject->pose.pos = vector3(0, 0, 0);
  }

  unsigned int currentLevel = objects.size() - 1;

  do {
    currentLevel--;
    SpatialRelationType relationType = relations[currentLevel];
    spatial::Object *onObject = objects[currentLevel];
    supportObject = objects[currentLevel + 1];

    string onObjectLabel = objectLabels[currentLevel];

    // Identify support object by "label+length+width"
    //    string supportObjectLabel = objectLabels[currentLevel+1];
    string supportObjectLabel;
    ostringstream ostr(supportObjectLabel);
    if (supportObject->type == spatial::OBJECT_BOX || supportObject->type
        == spatial::OBJECT_HOLLOW_BOX) {
      ostr << (int) (100 * ((BoxObject*) supportObject)->radius1) << "x"
          << (int) (100 * ((BoxObject*) supportObject)->radius2);
    } else {
      printf("Error! %s, %i: Unexpected object type!", __FILE__, __LINE__);
      return;
    }

    Pose3 oldPose = onObject->pose;

    SampleCloud *cloud = 0;

    // If the support object here is the "given" object - i.e. the bottommost one
    // in the hierarchy, and that pose is known,
    // then create a temporary sample cloud for that combination
    // of orientations. Otherwise, see if there's one cached
    if (currentLevel + 1 == objects.size() - 1 && !randomizeBaseObjectPose) {
      if (m_objectOrientations.find(onObjectLabel)
          == m_objectOrientations.end()) {
        if (!tryLoadOrientationsFromFile(onObjectLabel)) {
          getRandomSampleSphere(m_objectOrientations[onObjectLabel],
              m_orientationQuantization);
          writeOrientationsToFile(onObjectLabel);
        }
      }

      vector<Matrix33> supportObjectOrientations;
      supportObjectOrientations.push_back(supportObject->pose.rot);

      // Have to create new cloud
      cloud = createRelativeSampleCloud(relationType, supportObject, onObject,
          supportObjectOrientations, m_objectOrientations[onObjectLabel],
          cellSize);
      cloud->compute(*m_evaluator);
    } else {
      // Randomly sampled orientations on both objects
      // See if there's a cached point cloud for this
      for (vector<SampleCloudContainer>::iterator it = m_sampleClouds.begin(); it
          != m_sampleClouds.end(); it++) {
        if (it->obj1Label == supportObjectLabel && it->obj2Label
            == onObjectLabel && it->cloud->rel == relationType) {
          cloud = it->cloud;
          break;
        }
      }
      if (cloud == 0) {
        // Could not find this cloud. See if there's one on disk
        cloud = tryLoadCloudFromFile(supportObjectLabel, onObjectLabel,
            relationType);
        if (cloud == 0) {
          // Failed to load cloud from file
          cout << "failed to get cloud doing something else.." << endl;
          // Randomize orientations for involved objects, unless we already have such
          if (m_objectOrientations.find(supportObjectLabel)
              == m_objectOrientations.end()) {
            if (!tryLoadOrientationsFromFile(supportObjectLabel)) {
              if (supportObject->type == OBJECT_PLANE ||
              // FIXME
                  supportObjectLabel == "table" || supportObjectLabel
                  == "table1" || supportObjectLabel == "table2"
                  || supportObjectLabel == "bookcase_sm" || supportObjectLabel
                  == "bookcase_lg" || supportObjectLabel == "shelves"
                  || supportObjectLabel == "desk") {
                getRandomSampleCircle(m_objectOrientations[supportObjectLabel],
                    m_orientationQuantization);
              } else {
                getRandomSampleSphere(m_objectOrientations[supportObjectLabel],
                    m_orientationQuantization);
              }
              writeOrientationsToFile(supportObjectLabel);
            }
          }
          if (m_objectOrientations.find(onObjectLabel)
              == m_objectOrientations.end()) {
            if (!tryLoadOrientationsFromFile(onObjectLabel)) {
              getRandomSampleSphere(m_objectOrientations[onObjectLabel],
                  m_orientationQuantization);
              writeOrientationsToFile(onObjectLabel);
            }
          }

          // Have to create new cloud
          cloud = createRelativeSampleCloud(relationType, supportObject,
              onObject, m_objectOrientations[supportObjectLabel],
              m_objectOrientations[onObjectLabel], cellSize);
          cloud->compute(*m_evaluator);

          // Save it for next time
          writeCloudToFile(cloud, supportObjectLabel, onObjectLabel,
              relationType);
        }
        cout << "pushing cloud to list" << endl;
        SampleCloudContainer tmp = { cloud, supportObjectLabel, onObjectLabel };
        m_sampleClouds.push_back(tmp);
      }
    }

    //    log("Computing relation: %i", currentLevel);
    if (currentLevel == objects.size() - 2) {
      outCloud = *cloud;
    } else {
      //      log("Compositing relations");
      outCloud = outCloud.composit(*cloud);
    }

  } while (currentLevel > 0);
}

SampleCloud SampleCloud::composit(const SampleCloud &B) const {
  const SampleCloud &A = *this;

  if (A.sampleIntervalQuantum != B.sampleIntervalQuantum) {
    cerr << "Interval length mismatch!";
    exit(1);
  }

  cout << "Compositing...\n";

  //Find largest common factor between interval multipliers
  int maxMult, minMult;
  if (A.sampleIntervalMultiplier > B.sampleIntervalMultiplier) {
    maxMult = A.sampleIntervalMultiplier;
    minMult = B.sampleIntervalMultiplier;
  } else {
    maxMult = B.sampleIntervalMultiplier;
    minMult = A.sampleIntervalMultiplier;
  }
  int i;
  for (i = minMult; i > 1; i--) {
    if (maxMult % i == 0 && minMult % i == 0)
      break;
  }

  int newXExtent = A.xExtent * A.sampleIntervalMultiplier + B.xExtent
      * B.sampleIntervalMultiplier;
  int newYExtent = A.yExtent * A.sampleIntervalMultiplier + B.yExtent
      * B.sampleIntervalMultiplier;
  int newZExtent = A.zExtent * A.sampleIntervalMultiplier + B.zExtent
      * B.sampleIntervalMultiplier;
  newXExtent /= i;
  newYExtent /= i;
  newZExtent /= i;

  SampleCloud C(A.object1, B.object2, RELATION_COMPOSITE, A.sampleOffset
      + B.sampleOffset, A.sampleIntervalQuantum, i, newXExtent, newYExtent,
      newZExtent, A.object1Orientations, B.object2Orientations);
  C.kernelRadius = max(A.kernelRadius, B.kernelRadius);

  long numberOfValuesInC = (2 * newXExtent + 1) * (2 * newYExtent + 1) * (2
      * newZExtent + 1) * A.object1Orientations.size()
      * B.object2Orientations.size();

  C.values.resize(numberOfValuesInC); //Ooof!

  int AStepInC = A.sampleIntervalMultiplier / C.sampleIntervalMultiplier;
  int BStepInC = B.sampleIntervalMultiplier / C.sampleIntervalMultiplier;

  //  int AXL = 2 * A.xExtent + 1;
  int AYL = 2 * A.yExtent + 1;
  int AZL = 2 * A.zExtent + 1;
  //  int BXL = 2 * B.xExtent + 1;
  int BYL = 2 * B.yExtent + 1;
  int BZL = 2 * B.zExtent + 1;
  //  int CXL = 2 * C.xExtent + 1;
  int CYL = 2 * C.yExtent + 1;
  int CZL = 2 * C.zExtent + 1;

  const int nOrientations1 = A.object1Orientations.size();
  const int nOrientations2 = B.object2Orientations.size();
  const int nSummedOrientations = A.object2Orientations.size();

  const int valuesPer3DPoint = nOrientations1 * nOrientations2;

  // Now, loop over all the sample points in A

  long samplesInA = AYL * AZL * (2 * A.xExtent + 1);
  long samplesInB = BYL * BZL * (2 * B.xExtent + 1);

  double tmp[1000]; //FIXME
  int offsetInAColumn = 0;
  int offsetInA_ZY = 0;
  int offsetInBColumn = 0;
  int offsetInB_ZY = 0;
  for (int a = 0, ca = 0; a < samplesInA; a++, ca += AStepInC) {
    long pointOffsetInA = valuesPer3DPoint * a;
    for (int b = 0, cb = 0; b < samplesInB; b++, cb += BStepInC) {
      long pointOffsetInB = valuesPer3DPoint * b;
      long pointOffsetInC = valuesPer3DPoint * (ca + cb);

      for (int i = 0; i < nOrientations1; i++) {
        for (int k = 0; k < nSummedOrientations; k++) {
          tmp[k] = A.values[pointOffsetInA + nSummedOrientations * i + k];
        }
        for (int j = 0; j < nOrientations2; j++) {
          double sum = 0.0;
          for (int k = 0; k < nSummedOrientations; k++) {
            sum += tmp[k] * B.values[pointOffsetInB + nOrientations2 * k + j];
          }
          C.values[pointOffsetInC + nOrientations2 * i + j] += sum;
        }
      }

      offsetInBColumn++;
      if (offsetInBColumn == BZL) {
        // If we're skipping columns in B, compensate
        cb += BStepInC * (CZL - BZL);
        offsetInBColumn = 0;
        offsetInB_ZY++;
      }
      if (offsetInB_ZY == BYL) {
        // If we're skipping Y levels, compensate
        cb += BStepInC * CZL * (CYL - BYL);
        offsetInB_ZY = 0;
      }
    }

    offsetInAColumn++;
    if (offsetInAColumn == AZL) {
      // If we're skipping columns in A, compensate
      ca += AStepInC * (CZL - AZL);
      offsetInAColumn = 0;
      offsetInA_ZY++;
    }
    if (offsetInA_ZY == AYL) {
      // If we're skipping Y levels, compensate
      ca += AStepInC * CZL * (CYL - AYL);
      offsetInA_ZY = 0;
    }
    cout << (a * 100) / samplesInA << "%...    ";
    cout.flush();
  }
  cout << endl;

  return C;
}

void SampleCloud::compute(RelationEvaluator &evaluator) {
  // Fill the values vector with values for the relation in question, 
  // for each relative position, and each combination of orientations
  // for the two objects.

  // First make sure the values vector has the right size
  long numberOfValues = (2 * xExtent + 1) * (2 * yExtent + 1) * (2 * zExtent
      + 1) * object1Orientations.size() * object2Orientations.size();

  cout << "Computing point cloud...";

  values.resize(numberOfValues); //Ooof!

  // Loop over relative positions
  double sampleInterval = sampleIntervalQuantum * sampleIntervalMultiplier;
  vector<double>::iterator it = values.begin();
  double minX = sampleOffset.x - xExtent * sampleInterval;
  double minY = sampleOffset.y - yExtent * sampleInterval;
  double minZ = sampleOffset.z - zExtent * sampleInterval;

  double x = minX, y, z;
  object1->pose.pos = vector3(0, 0, 0);
  std::set<int> orientationsOverThreshold;
  //  long samplesOverThreshold = 0;
  switch (rel) {
  case RELATION_ON:
    for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
      y = minY;
      for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
        z = minZ;
        for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
          object2->pose.pos = vector3(x, y, z);

          // Loop over combinations of orientations
          for (vector<Matrix33>::iterator o1it = object1Orientations.begin(); o1it
              != object1Orientations.end(); o1it++) {
            object1->pose.rot = *o1it;
            int i = 0;
            for (vector<Matrix33>::iterator o2it = object2Orientations.begin(); o2it
                != object2Orientations.end(); o2it++) {
              object2->pose.rot = *o2it;
              (*it) = evaluator.evaluateOnness(object1, object2);
              //		if (*it > 0.5) {
              //		  orientationsOverThreshold.insert(i);
              //		  samplesOverThreshold++;
              //		}
              it++;
              i++;
            }
          }
        }
      }
      cout << (100 * (xi + xExtent + 1)) / (2 * xExtent + 1) << "%...    ";
      cout.flush();
    }
    cout << endl;
    break;
  case RELATION_IN:
    for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
      y = minY;
      for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
        z = minZ;
        for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
          object2->pose.pos = vector3(x, y, z);

          // Loop over combinations of orientations
          for (vector<Matrix33>::iterator o1it = object1Orientations.begin(); o1it
              != object1Orientations.end(); o1it++) {
            object1->pose.rot = *o1it;
            for (vector<Matrix33>::iterator o2it = object2Orientations.begin(); o2it
                != object2Orientations.end(); o2it++) {
              object2->pose.rot = *o2it;
              (*it) = evaluator.evaluateInness(object1, object2);
              it++;
            }
          }
        }
      }
      cout << (100 * (xi + xExtent + 1)) / (2 * xExtent + 1) << "%...    ";
      cout.flush();
    }
    cout << endl;
    break;
  default:
    cerr << "Error! Unsupported relation\n";
    exit(1);
  }
}

void SampleCloud::makePointCloud(Vector3 &center, double &interval, int &xExt,
    int &yExt, int &zExt, vector<double> &weights) const {
  center = sampleOffset;
  interval = sampleIntervalQuantum * sampleIntervalMultiplier;
  xExt = xExtent;
  yExt = yExtent;
  zExt = zExtent;

  weights.resize(values.size());

  unsigned long i = 0;
  for (int xi = -xExtent; xi <= xExtent; xi++) {
    for (int yi = -yExtent; yi <= yExtent; yi++) {
      for (int zi = -zExtent; zi <= zExtent; zi++) {
        weights[i] = values[i];
        i++;
      }
    }
  }
}

void SampleCloud::KernelDensityEstimation2D(Cure::LocalGridMap<double> &outMap,
    Vector3 cloudCenter, double kernelWidthFactor, double &total,
    double baseValue) {
  double sampleInterval = sampleIntervalQuantum * sampleIntervalMultiplier;
  double cellSize = outMap.getCellSize();
  int mapSize = outMap.getSize();

  if (kernelRadius == 0.0)
    kernelRadius = sampleInterval * kernelWidthFactor;
  if (kernelRadius < cellSize)
    kernelRadius = cellSize;
  int kernelWidth = (int) (ceil(kernelRadius / cellSize) + 0.1);
  //How many kernelRadius:es per cell
  double kernelStep = cellSize / kernelRadius;

  //Put together a kernel with this width, offset by sampleOffset
  OffsetKernel kernel;

  Vector3 sampleCenter = cloudCenter + sampleOffset;

  double sx = sampleCenter.x;
  double sy = sampleCenter.y;
  //  double sz = sampleCenter.z;

  int sampleMapX, sampleMapY;
  outMap.worldCoords2Index(sx, sy, sampleMapX, sampleMapY);

  kernel.minxi = -kernelWidth; //Relative indices of kernel patch extents
  kernel.minyi = -kernelWidth;
  kernel.maxxi = +kernelWidth;
  kernel.maxyi = +kernelWidth;

  double minx = (sx - kernelWidth * cellSize) / kernelRadius; //Note: Normalized coordinates
  double miny = (sy - kernelWidth * cellSize) / kernelRadius; //(relative to kernel radius)

  double sumWeights = 0.0;
  double x = minx;
  for (int xi = kernel.minxi; xi <= kernel.maxxi; xi++, x += kernelStep) {
    double y = miny;
    for (int yi = kernel.minyi; yi <= kernel.maxyi; yi++, y += kernelStep) {
      double sq = x * x + y * y;
      //double sqsum = 1 - sq;
      double sqsum = (1 - sq) * (1 - sq) * (1 - sq);
      if (sqsum > 0) {
        kernel.weights.push_back(sqsum);
        sumWeights += sqsum;
      } else {
        kernel.weights.push_back(0);
      }
    }
  }
  for (unsigned int i = 0; i < kernel.weights.size(); i++) {
    kernel.weights[i] = kernel.weights[i] / sumWeights;
  }

  //Kernel done, now KDE the heck out of it!

  vector<double>::iterator it = values.begin();
  double minX = sampleOffset.x - xExtent * sampleInterval;
  double minY = sampleOffset.y - yExtent * sampleInterval;
  double minZ = sampleOffset.z - zExtent * sampleInterval;

  const unsigned int totalOrientationCombinations = object2Orientations.size()
      * object1Orientations.size();

  x = minX;
  // Loop over samples in cloud
  for (int xi = -xExtent; xi <= xExtent; xi++, x += sampleInterval) {
    double y = minY;
    for (int yi = -yExtent; yi <= yExtent; yi++, y += sampleInterval) {
      double z = minZ;
      //Sum up all weight inside a column of samples before accumulating the kernel
      double totalValueThisXY = 0.0;
      for (int zi = -zExtent; zi <= zExtent; zi++, z += sampleInterval) {
        // Loop over combinations of orientations
        for (unsigned int i = 0; i < totalOrientationCombinations; i++) {
          totalValueThisXY += *it;
          it++;
        }
      }

      // Loop over kernel
      int kernelCellNo = 0;
      int kX = sampleMapX + xi * sampleIntervalMultiplier;
      int kY = sampleMapY + yi * sampleIntervalMultiplier;
      for (int kxi = kX + kernel.minxi; kxi <= kX + kernel.maxxi; kxi++) {
        for (int kyi = kY + kernel.minyi; kyi <= kY + kernel.maxyi; kyi++, kernelCellNo++) {
          if (kxi <= -mapSize || kxi >= mapSize || kyi <= -mapSize || kyi
              >= mapSize)
            continue;

          outMap(kxi, kyi) += kernel.weights[kernelCellNo] * totalValueThisXY
              * baseValue;
          total += kernel.weights[kernelCellNo] * totalValueThisXY * baseValue;
        }
      }
    }
  }
}

template<class T>
void writeBinary(ostream &o, const T& item) {
  const char *tmp = reinterpret_cast<const char *> (&item);
  o.write(tmp, sizeof(T));
}

template<class T>
void readBinary(istream &o, T& item) {
  char buf[sizeof(T)];
  o.read(buf, sizeof(T));
  item = *(reinterpret_cast<T *> (buf));
}

ostream &operator<<(ostream &o, const SampleCloud &c) {
  writeBinary(o, c.sampleOffset);
  writeBinary(o, c.sampleIntervalQuantum);
  writeBinary(o, c.sampleIntervalMultiplier);
  writeBinary(o, c.xExtent);
  writeBinary(o, c.yExtent);
  writeBinary(o, c.zExtent);

  writeBinary(o, c.object1Orientations.size());
  writeBinary(o, c.object2Orientations.size());
  for (unsigned long i = 0; i < c.object1Orientations.size(); i++) {
    writeBinary(o, c.object1Orientations[i]);
  }
  for (unsigned long i = 0; i < c.object2Orientations.size(); i++)
    writeBinary(o, c.object2Orientations[i]);

  const long numberOfValues = (2 * c.xExtent + 1) * (2 * c.yExtent + 1) * (2
      * c.zExtent + 1) * c.object1Orientations.size()
      * c.object2Orientations.size();

  for (long i = 0; i < numberOfValues; i++) {
    writeBinary(o, c.values[i]);
  }

  writeBinary(o, c.object1->type);
  switch (c.object1->type) {
  case OBJECT_PLANE:
    writeBinary(o, *(static_cast<PlaneObject *> (c.object1)));
    break;
  case OBJECT_BOX:
    writeBinary(o, *(static_cast<BoxObject *> (c.object1)));
    break;
  case OBJECT_CYLINDER:
    //      writeBinary(o, *(static_cast< *>(c.object1)));
    break;
  case OBJECT_SPHERE:
    //      writeBinary(o, *(static_cast<PlaneObject *>(c.object1)));
    break;
  case OBJECT_HOLLOW_BOX:
    writeBinary(o, *(static_cast<HollowBoxObject *> (c.object1)));
    break;
  }

  writeBinary(o, c.object2->type);
  switch (c.object2->type) {
  case OBJECT_PLANE:
    writeBinary(o, *(static_cast<PlaneObject *> (c.object2)));
    break;
  case OBJECT_BOX:
    writeBinary(o, *(static_cast<BoxObject *> (c.object2)));
    break;
  case OBJECT_CYLINDER:
    //      writeBinary(o, *(static_cast< *>(c.object2)));
    break;
  case OBJECT_SPHERE:
    //      writeBinary(o, *(static_cast<PlaneObject *>(c.object2)));
    break;
  case OBJECT_HOLLOW_BOX:
    writeBinary(o, *(static_cast<HollowBoxObject *> (c.object2)));
    break;
  }

  return o;
}

istream &operator>>(istream &o, SampleCloud &c) {
  readBinary(o, c.sampleOffset);
  readBinary(o, c.sampleIntervalQuantum);
  readBinary(o, c.sampleIntervalMultiplier);
  readBinary(o, c.xExtent);
  readBinary(o, c.yExtent);
  readBinary(o, c.zExtent);

  size_t nOri1;
  readBinary(o, nOri1);
  size_t nOri2;
  readBinary(o, nOri2);
  c.object1Orientations.resize(nOri1);
  c.object2Orientations.resize(nOri2);

  for (unsigned long i = 0; i < nOri1; i++) {
    readBinary(o, c.object1Orientations[i]);
  }
  for (unsigned long i = 0; i < nOri2; i++) {
    readBinary(o, c.object2Orientations[i]);
  }

  const long numberOfValues = (2 * c.xExtent + 1) * (2 * c.yExtent + 1) * (2
      * c.zExtent + 1) * c.object1Orientations.size()
      * c.object2Orientations.size();
  c.values.resize(numberOfValues);
  for (long i = 0; i < numberOfValues; i++)
    readBinary(o, c.values[i]);

  SpatialObjectType type;
  readBinary(o, type);
  switch (type) {
  case OBJECT_PLANE: {
    PlaneObject *ptr = new PlaneObject;
    readBinary(o, *ptr);
    c.object1 = ptr;
    break;
  }
  case OBJECT_BOX: {
    BoxObject *ptr = new BoxObject;
    readBinary(o, *ptr);
    c.object1 = ptr;
    break;
  }
    break;
  case OBJECT_CYLINDER: {
    //	CylinderObject *ptr = new CylinderObject;
    //	readBinary(o, *ptr);
    //	c.object1 = ptr;
    break;
  }
    break;
  case OBJECT_SPHERE: {
    //	SphereObject *ptr = new SphereObject;
    //	readBinary(o, *ptr);
    //	c.object1 = ptr;
    break;
  }
    break;
  case OBJECT_HOLLOW_BOX: {
    HollowBoxObject *ptr = new HollowBoxObject;
    readBinary(o, *ptr);
    c.object1 = ptr;
    break;
  }
    break;
  }

  readBinary(o, type);
  switch (type) {
  case OBJECT_PLANE: {
    PlaneObject *ptr = new PlaneObject;
    readBinary(o, *ptr);
    c.object2 = ptr;
    break;
  }
  case OBJECT_BOX: {
    BoxObject *ptr = new BoxObject;
    readBinary(o, *ptr);
    c.object2 = ptr;
    break;
  }
    break;
  case OBJECT_CYLINDER: {
    //	CylinderObject *ptr = new CylinderObject;
    //	readBinary(o, *ptr);
    //	c.object2 = ptr;
    break;
  }
    break;
  case OBJECT_SPHERE: {
    //	SphereObject *ptr = new SphereObject;
    //	readBinary(o, *ptr);
    //	c.object2 = ptr;
    break;
  }
    break;
  case OBJECT_HOLLOW_BOX: {
    HollowBoxObject *ptr = new HollowBoxObject;
    readBinary(o, *ptr);
    c.object2 = ptr;
    break;
  }
    break;
  }

  return o;
}

SampleCloud *
DensitySampler::tryLoadCloudFromFile(const string &supportObjectLabel,
    const string &onObjectLabel, SpatialRelationType type) {
  cerr << "Not going to try clouds from file returning.." << std::endl;
  return 0;
  string filename("cloud_");
  filename += onObjectLabel;
  switch (type) {
  case RELATION_ON:
    filename += "+on+";
    break;
  case RELATION_IN:
    filename += "+in+";
    break;
  default:
    cerr << "Error! Loading unknown relation type\n";
    exit(1);
  }
  filename += supportObjectLabel;
  filename += ".cld";

  ifstream infile(filename.c_str(), ios::in | ios::binary);

  if (!infile.good()) {
    return 0;
  }

  cout << "Attempting to load sample point cloud from file " << filename
      << endl;

  SampleCloud *cloud = new SampleCloud();
  try {
    infile >> *cloud;
  } catch (...) {
    cerr << "Error on file read!\n";
    exit(1);
  }

  // Check that the orientations stored are the same as we're using, if any
  if (m_objectOrientations.find(supportObjectLabel)
      != m_objectOrientations.end()) {
    if (m_objectOrientations[supportObjectLabel].size()
        != cloud->object1Orientations.size()) {
      cerr
          << "Error! point cloud loaded from file has different orientations than already present in system!\n";
      exit(1);
    }
    for (unsigned int i = 0; i
        < m_objectOrientations[supportObjectLabel].size(); i++) {
      if (!isZero(m_objectOrientations[supportObjectLabel][i]
          - cloud->object1Orientations[i])) {
        cerr
            << "Error! point cloud loaded from file has different orientations than already present in system!\n";
        exit(1);
      }
    }
  }
  m_objectOrientations[supportObjectLabel] = cloud->object1Orientations;

  if (m_objectOrientations.find(onObjectLabel) != m_objectOrientations.end()) {
    if (m_objectOrientations[onObjectLabel].size()
        != cloud->object2Orientations.size()) {
      cerr
          << "Error! point cloud loaded from file has different orientations than already present in system!\n";
      exit(1);
    }
    for (unsigned int i = 0; i < m_objectOrientations[onObjectLabel].size(); i++) {
      if (!isZero(m_objectOrientations[onObjectLabel][i]
          - cloud->object2Orientations[i])) {
        cerr
            << "Error! point cloud loaded from file has different orientations than already present in system!\n";
        exit(1);
      }
    }
  }
  m_objectOrientations[onObjectLabel] = cloud->object2Orientations;
  cout << "...load successful\n";

  return cloud;
}

void SampleCloud::compact() {
  const int nOrientations1 = object1Orientations.size();
  const int nOrientations2 = object2Orientations.size();

  const int valuesPer3DPoint = nOrientations1 * nOrientations2;
  unsigned long totalValues = values.size();

  for (unsigned int i1 = 0, i2 = 0; i2 < totalValues; i1++, i2
      += valuesPer3DPoint) {
    double sumThisXYZ = 0.0;
    for (unsigned int i3 = i2; i3 < i2 + valuesPer3DPoint; i3++)
      sumThisXYZ += values[i3];
    values[i1] = sumThisXYZ;
  }
}

void DensitySampler::writeCloudToFile(const SampleCloud *cloud,
    const string &supportObjectLabel, const string &onObjectLabel,
    SpatialRelationType type) {
  string filename("cloud_");
  filename += onObjectLabel;
  switch (type) {
  case RELATION_ON:
    filename += "+on+";
    break;
  case RELATION_IN:
    filename += "+in+";
    break;
  default:
    cerr << "Error! Loading unknown relation type\n";
    exit(1);
  }
  filename += supportObjectLabel;
  filename += ".cld";

  cout << "Writing sample cloud to file: " << filename << endl;
  ofstream outfile(filename.c_str(), ios::out | ios::binary);

  if (!outfile.good()) {
    cerr << "Unable to write to file!\n";
    exit(1);
  }

  try {
    outfile << *cloud;
  } catch (...) {
    cerr << "Error writing to file!\n";
    exit(1);
  }
}

bool DensitySampler::tryLoadOrientationsFromFile(const string &label) {
  string filename("orientations_");
  filename += label;
  filename += ".cld";

  ifstream infile(filename.c_str(), ios::in | ios::binary);

  if (!infile.good()) {
    return false;
  }

  try {
    size_t nOri;
    readBinary(infile, nOri);
    m_objectOrientations[label].resize(nOri);
    for (unsigned int i = 0; i < nOri; i++) {
      readBinary(infile, m_objectOrientations[label][i]);
    }
  } catch (...) {
    cerr << "Error reading orientation file!\n";
    exit(1);
  }
  return true;
}

double DensitySampler::kernelDensityEstimation3D(SpatialGridMap::GridMap<
    SpatialGridMap::GridMapData> &map,
    const vector<cogx::Math::Vector3> &centers, double interval, int xExtent,
    int yExtent, int zExtent, const vector<double> &values,
    double baseMultiplier, double totalWeight, const Cure::LocalGridMap<
        unsigned char> *lgm) {
  double cellSize = map.getCellSize();
  double kernelRadius = interval;
  if (kernelRadius < cellSize)
    kernelRadius = cellSize;

  //Get outer limits of update region (in xy)
  //  const pair<int, int> mapCenter =
  //    map.worldToGridCoords(center.x, center.y);
  const pair<int, int> mapMin = make_pair<int, int> (0, 0);
  //    map.worldToGridCoords(center.x - xExtent*interval - kernelRadius, 
  //	center.y - yExtent*interval - kernelRadius);
  const pair<int, int> mapMax = make_pair<int, int> (
      map.getMapSize().first - 1, map.getMapSize().second - 1);
  //    map.worldToGridCoords(center.x + xExtent*interval + kernelRadius, 
  //	center.y + yExtent*interval + kernelRadius);

  double total = 0.0;
  // Loop over columns in bounding box
  cout << "Finding correct KDE weight...\n";

  for (int x = mapMin.first; x <= mapMax.first; x++) {
    cout << (100 * x) / (mapMax.first - mapMin.first + 1) << "% ";
    cout.flush();
    for (int y = mapMin.second; y <= mapMax.second; y++) {
      // Column center world coords
      pair<double, double> columnWorldXY = map.gridToWorldCoords(x, y);

      //      // If we're supplied with a lgm, and the column is in unknown space,
      //      // skip it
      //      if (lgm != 0) {
      //	int lgmX, lgmY;
      //	lgm->worldCoords2Index(columnWorldXY.first, columnWorldXY.second, lgmX, lgmY);
      //	if ((*lgm)(lgmX, lgmY) == '2') 
      //	  continue;
      //      }

      vector<vector<double> > sampleZValues;
      vector<vector<double> > sampleWeights;
      vector<double> columnXYSqDiffs;

      double minZ = DBL_MAX;
      double maxZ = -DBL_MAX;
      for (vector<Vector3>::const_iterator sampleIt = centers.begin(); sampleIt
          != centers.end(); sampleIt++) {
        const Vector3 &center = *sampleIt;
        const double lowerZ = center.z - zExtent * interval - kernelRadius;
        if (lowerZ < minZ)
          minZ = lowerZ;
        const double upperZ = center.z + zExtent * interval + kernelRadius;
        if (upperZ > maxZ)
          maxZ = upperZ;

        for (int sampleX = -xExtent; sampleX <= xExtent; sampleX++) {
          double wsx = sampleX * interval + center.x;
          double xDiff = (wsx - columnWorldXY.first) / kernelRadius;
          if (xDiff < 1 && xDiff > -1) {
            for (int sampleY = -yExtent; sampleY <= yExtent; sampleY++) {
              double wsy = sampleY * interval + center.y;
              double yDiff = (wsy - columnWorldXY.second) / kernelRadius;
              if (yDiff < 1 && yDiff > -1) {
                columnXYSqDiffs.push_back(xDiff * xDiff + yDiff * yDiff);
                sampleZValues.push_back(vector<double> ());
                sampleWeights.push_back(vector<double> ());

                unsigned int indexOffset = (2 * zExtent + 1) * (sampleY
                    + yExtent + (2 * yExtent + 1) * (sampleX + xExtent));
                for (int sampleZ = -zExtent; sampleZ <= zExtent; sampleZ++) {
                  sampleZValues.back().push_back(center.z + interval * sampleZ);
                  sampleWeights.back().push_back(values[indexOffset + sampleZ
                      + zExtent]);
                }
                //	      map.boxSubColumnModifier(x,y,minZ,maxZ, columnKDEFunctor);
              }
            }
          }
        }
      }
      SpatialGridMap::KDEVolumeQuery volumeFunctor(kernelRadius, sampleZValues,
          sampleWeights, columnXYSqDiffs);
      map.alignedBoxQuery(x, x, y, y, minZ, maxZ, volumeFunctor, false);
      total += volumeFunctor.getTotal();
    }
  }

  cout << "\nSummed KDE adjusted volume: " << total;

  const double adjustmentMultiplier = totalWeight / total;

  total = 0.0;

  cout << "\nApplying KDE... ";
  for (int x = mapMin.first; x <= mapMax.first; x++) {
    cout << (100 * x) / (mapMax.first - mapMin.first + 1) << "% ";
    for (int y = mapMin.second; y <= mapMax.second; y++) {
      // Column center world coords
      pair<double, double> columnWorldXY = map.gridToWorldCoords(x, y);

      vector<vector<double> > sampleZValues;
      vector<vector<double> > sampleWeights;
      vector<double> columnXYSqDiffs;

      //      // If we're supplied with a lgm, and the column is in unknown space,
      //      // skip it
      //      if (lgm != 0) {
      //	int lgmX, lgmY;
      //	lgm->worldCoords2Index(columnWorldXY.first, columnWorldXY.second, lgmX, lgmY);
      //	if ((*lgm)(lgmX, lgmY) == '2') 
      //	  continue;
      //      }

      double minZ = DBL_MAX;
      double maxZ = -DBL_MAX;
      for (vector<Vector3>::const_iterator sampleIt = centers.begin(); sampleIt
          != centers.end(); sampleIt++) {
        const Vector3 &center = *sampleIt;
        const double lowerZ = center.z - zExtent * interval - kernelRadius;
        if (lowerZ < minZ)
          minZ = lowerZ;
        const double upperZ = center.z + zExtent * interval + kernelRadius;
        if (upperZ > maxZ)
          maxZ = upperZ;

        for (int sampleX = -xExtent; sampleX <= xExtent; sampleX++) {
          double wsx = sampleX * interval + center.x;
          double xDiff = (wsx - columnWorldXY.first) / kernelRadius;
          if (xDiff < 1 && xDiff > -1) {
            for (int sampleY = -yExtent; sampleY <= yExtent; sampleY++) {
              double wsy = sampleY * interval + center.y;
              double yDiff = (wsy - columnWorldXY.second) / kernelRadius;
              if (yDiff < 1 && yDiff > -1) {
                columnXYSqDiffs.push_back(xDiff * xDiff + yDiff * yDiff);
                sampleZValues.push_back(vector<double> ());
                sampleWeights.push_back(vector<double> ());

                unsigned int indexOffset = (2 * zExtent + 1) * (sampleY
                    + yExtent + (2 * yExtent + 1) * (sampleX + xExtent));
                for (int sampleZ = -zExtent; sampleZ <= zExtent; sampleZ++) {
                  sampleZValues.back().push_back(center.z + interval * sampleZ);
                  sampleWeights.back().push_back(values[indexOffset + sampleZ
                      + zExtent]);
                }
              }
            }
          }
        }
      }
      SpatialGridMap::GDAddKDE columnKDEFunctor(kernelRadius, sampleZValues,
          sampleWeights, columnXYSqDiffs, adjustmentMultiplier);

      map.alignedBoxQuery(x, x, y, y, minZ, maxZ, columnKDEFunctor, false);
      //		map.boxSubColumnModifier(x,y,minZ,maxZ, columnKDEFunctor);
      total += columnKDEFunctor.getTotal();
      map.tryMergeColumn(x, y);
    }
  }

  cout << "Total adjustment: " << total << endl;
  return total;
}

void DensitySampler::writeOrientationsToFile(const string &label) {
  string filename("orientations_");
  filename += label;
  filename += ".cld";

  ofstream outfile(filename.c_str(), ios::out | ios::binary);

  if (!outfile.good()) {
    cerr << "Unable to read file!\n";
    exit(1);
  }

  try {
    writeBinary(outfile, m_objectOrientations[label].size());
    for (unsigned int i = 0; i < m_objectOrientations[label].size(); i++) {
      writeBinary(outfile, m_objectOrientations[label][i]);
    }
  } catch (...) {
    cerr << "Error writing orientation file!\n";
    exit(1);
  }
}
}
