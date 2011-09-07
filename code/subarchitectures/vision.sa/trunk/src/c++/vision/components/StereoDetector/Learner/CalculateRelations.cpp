/**
 * @file CalculateRelations.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#include "CalculateRelations.h"

namespace Z
{
  
/**
 * @brief Constructor of class CalculateRelations.
 */
CalculateRelations::CalculateRelations()
{}

/**
 * @brief Constructor of class CalculateRelations.
 */
void CalculateRelations::Initialize(KinectCore *k, double fx, double fy, double cx, double cy)
{
  kcore = k;
  cam_fx = fx;
  cam_fy = fy;
  cam_cx = cx;
  cam_cy = cy;
  relations.resize(0);
}


/**
 * @brief Calculate relations using the ground-truth data to get positive and
 * negative examples for the SVM training.
 * @param rel Relations vector
 */
void CalculateRelations::CalcSVMRelations(std::vector<Relation> &rel)
{
  relations.resize(0);
  CalcSVMPatchPatchRelations();
  CalcSVMPatchLineRelations();
  CalcSVMLineLineRelations();
  rel = relations;
}


/**
 * @brief Calculate relations between 3D-patches using the ground-truth data, 
 * to get positive and negative examples for the SVM training.
 */
void CalculateRelations::CalcSVMPatchPatchRelations()
{
//   std::vector<double> positive, negative;
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
// printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
      
      // check if it belongs to the same object
      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
      {
        // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double normal_angle, plane_distance; 
        p0->CalculateCoplanarity(px, normal_angle, plane_distance);
        
// printf("  => true example!\n");
        Relation r;
        r.groundTruth = 1;
        r.prediction = -1;
        r.type = 1;
        r.id_0 = p0->GetNodeID();
        r.id_1 = px->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(colorSimilarity);
        r.rel_value.push_back(normal_angle);
        r.rel_value.push_back(plane_distance);
        relations.push_back(r);
      }
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) &&
               p0->GetObjectLabel() != px->GetObjectLabel())
      {
                // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double normal_angle, plane_distance; 
        p0->CalculateCoplanarity(px, normal_angle, plane_distance);
        
// printf("  => false example!\n");
        Relation r;
        r.groundTruth = 0;
        r.prediction = -1;
        r.type = 1;
        r.id_0 = p0->GetNodeID();
        r.id_1 = px->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(colorSimilarity);
        r.rel_value.push_back(normal_angle);
        r.rel_value.push_back(plane_distance);
        relations.push_back(r);
      }
    }
  }
// printf("CalculateRelations::CalcPatchPatchRelations: allRelations: %u\n", allRelations.size());
//   rel = ppRelations;
}

/**
 * @brief Calculate relations between 3D-patches and lines using the ground-truth data, 
 * to get positive and negative examples for the SVM training.
 */
void CalculateRelations::CalcSVMPatchLineRelations()
{
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  unsigned nrLines = kcore->NumGestalts3D(Gestalt3D::LINE);
  
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=0; j<nrLines; j++)
    {
      Line3D *l = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, j);
      
// printf("%u-%u: prox: %4.3f - para: %4.3f \n", p->GetNodeID(), l->GetNodeID(), proximity, parallelity);
      
      // check if it belongs to the same object
      if(p->GetObjectLabel() != 0 && 
         l->GetObjectLabel() == p->GetObjectLabel() &&
         PLLineInPlaneROI(p, l))
      {
        double proximity = PLProximity(p, l);
        double parallelity = PLParallelity(p, l);
// printf("%u-%u: prox: %4.3f - para: %4.3f ", p->GetNodeID(), l->GetNodeID(), proximity, parallelity);
// printf("  => true example! (%u - %u)\n", p->GetObjectLabel(), l->GetObjectLabel());
        Relation r;
        r.groundTruth = 1;
        r.prediction = -1;
        r.type = 2;
        r.id_0 = p->GetNodeID();
        r.id_1 = l->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(parallelity);
        r.remove = false;
        relations.push_back(r);
      }
      else if((p->GetObjectLabel() != 0 || l->GetObjectLabel() != 0) &&
               p->GetObjectLabel() != l->GetObjectLabel())
      {
        double proximity = PLProximity(p, l);
        double parallelity = PLParallelity(p, l);
// printf("%u-%u: prox: %4.3f - para: %4.3f ", p->GetNodeID(), l->GetNodeID(), proximity, parallelity);
// printf("  => false example! (%u - %u)\n", p->GetObjectLabel(), l->GetObjectLabel());
        Relation r;
        r.groundTruth = 0;
        r.prediction = -1;
        r.type = 2;
        r.id_0 = p->GetNodeID();
        r.id_1 = l->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(parallelity);
        r.remove = false;
        relations.push_back(r);
      }
    }
  }
}

/**
 * @brief Calculate relations between 3D-lines using the ground-truth data, 
 * to get positive and negative examples for the SVM training.
 */
void CalculateRelations::CalcSVMLineLineRelations()
{
  unsigned nrLines = kcore->NumGestalts3D(Gestalt3D::LINE);
  for(unsigned i=0; i<nrLines; i++)
  {
    Line3D *l0 = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, i);
    for(unsigned j=i+1; j<nrLines; j++)
    {
      Line3D *l1 = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, j);
      
      if(l0->GetObjectLabel() != 0 && l0->GetObjectLabel() == l1->GetObjectLabel())
      {
        double proximity = l0->LLProximity(l1);
        double parallelity = l0->LLParallelity(l1);
// printf("%u-%u: (true) prox: %4.3f - para: %4.3f \n", l0->GetNodeID(), l1->GetNodeID(), proximity, parallelity);
// printf("%u-%u: prox: %4.3f - para: %4.3f ", p->GetNodeID(), l->GetNodeID(), proximity, parallelity);
// // printf("  => true example! (%u - %u)\n", p->GetObjectLabel(), l->GetObjectLabel());
        if(parallelity != -1)
        {
          Relation r;
          r.groundTruth = 1;
          r.prediction = -1;
          r.type = 3;
          r.id_0 = l0->GetNodeID();
          r.id_1 = l1->GetNodeID();
          r.rel_value.push_back(proximity);
          r.rel_value.push_back(parallelity);
          r.remove = false;
          relations.push_back(r);
        }
      }
      else if((l0->GetObjectLabel() != 0 || l1->GetObjectLabel() != 0) &&
               l0->GetObjectLabel() != l1->GetObjectLabel())
      {
        double proximity = l0->LLProximity(l1);
        double parallelity = l0->LLParallelity(l1);
// printf("%u-%u: (false) prox: %4.3f - para: %4.3f \n", l0->GetNodeID(), l1->GetNodeID(), proximity, parallelity);
// // printf("%u-%u: prox: %4.3f - para: %4.3f ", p->GetNodeID(), l->GetNodeID(), proximity, parallelity);
// // printf("  => false example! (%u - %u)\n", p->GetObjectLabel(), l->GetObjectLabel());
        if(parallelity != -1)
        {
          Relation r;
          r.groundTruth = 0;
          r.prediction = -1;
          r.type = 3;
          r.id_0 = l0->GetNodeID();
          r.id_1 = l1->GetNodeID();
          r.rel_value.push_back(proximity);
          r.rel_value.push_back(parallelity);
          r.remove = false;
          relations.push_back(r);
        }
      }
//       else printf("CalcSVMLineLineRelations: no result!\n");
    }
  }
}


/**
 * @brief Calculate all relations between 3D-patches for SVM prediction.
 * @param rel Relations vector
 */
void CalculateRelations::CalcTestRelations(std::vector<Relation> &rel)
{
  relations.resize(0);
  
printf("CalculateRelations::CalcTestRelations: Start\n");
  /// Patch-Patch relations
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
      if(p0->GetNodeID() != -1 && px->GetNodeID() != -1)
      {
  // printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
        // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double normal_angle, plane_distance; 
        p0->CalculateCoplanarity(px, normal_angle, plane_distance);
        
        Relation r;
        if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
          r.groundTruth = 1;
        else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) &&
                 p0->GetObjectLabel() != px->GetObjectLabel())
          r.groundTruth = 0;
        else
          r.groundTruth = -1;     /// TODO macht das Sinn???
        r.prediction = -1;
        r.type = 1;
        r.id_0 = p0->GetNodeID();
        r.id_1 = px->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(colorSimilarity);
        r.rel_value.push_back(normal_angle);
        r.rel_value.push_back(plane_distance);
        r.remove = false;
        relations.push_back(r); 
      }
    }
  }
printf("CalculateRelations::CalcTestRelations: Patch-Patch calculated!\n");
  
  /// Patch-Line relations 
  unsigned nrLines = kcore->NumGestalts3D(Gestalt3D::LINE);
  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=0; j<nrLines; j++)
    {
      Line3D *l = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, j);

      if(p->GetNodeID() != -1 && l->GetNodeID() != -1)
      {
        double proximity = PLProximity(p, l);
        double parallelity = PLParallelity(p, l);

        Relation r;
        if(p->GetObjectLabel() != 0 && 
          l->GetObjectLabel() == p->GetObjectLabel() &&
          PLLineInPlaneROI(p, l))     
          r.groundTruth = 1;
        else if((p->GetObjectLabel() != 0 || l->GetObjectLabel() != 0) &&
                p->GetObjectLabel() != l->GetObjectLabel())
          r.groundTruth = 0;
        else
          r.groundTruth = -1;
        r.prediction = -1;
        r.type = 2;
        r.id_0 = p->GetNodeID();
        r.id_1 = l->GetNodeID();
        r.rel_value.push_back(proximity);
        r.rel_value.push_back(parallelity);
        r.remove = false;
        relations.push_back(r);
      }
    }
  }
printf("CalculateRelations::CalcTestRelations: Patch-Line calculated!\n");

  /// Line-Line relations 
//   for(unsigned i=0; i<nrLines; i++)
//   {
//     Line3D *l0 = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, i);
//     for(unsigned j=i+1; j<nrLines; j++)
//     {
//       Line3D *l1 = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, j);
//       
//       double proximity = l0->LLProximity(l1);
//       double parallelity = l0->LLParallelity(l1);
// 
//       Relation r;
//       if(l0->GetObjectLabel() != 0 && 
//          l0->GetObjectLabel() == l1->GetObjectLabel() && parallelity != -1)
//         r.groundTruth = 1;
//       else if((l0->GetObjectLabel() != 0 || l1->GetObjectLabel() != 0) && 
//                l0->GetObjectLabel() != l1->GetObjectLabel() && parallelity != -1)
//         r.groundTruth = 0;
//       else r.groundTruth = -1;
//       r.prediction = -1;
//       r.type = 3;
//       r.id_0 = l0->GetNodeID();
//       r.id_1 = l1->GetNodeID();
//       r.rel_value.push_back(proximity);
//       r.rel_value.push_back(parallelity);
//       relations.push_back(r);
//     }
//   }
// printf("CalculateRelations::CalcTestRelations: Line-Line calculated!\n");

  rel = relations;
}


/**
 * @brief Calculate all relations between 3D-patches for SVM prediction.
 * @param rel Relations vector
 */
void CalculateRelations::CalcAllRelations(std::vector<Relation> &rel)
{
  /// calculate patch-patch relations
  std::vector<double> positive, negative;
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
// printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
      // calculate relation values
      double proximity = p0->CalculateProximity(px);
      double colorSimilarity = p0->CompareColor(px);
      double normal_angle, plane_distance; 
      p0->CalculateCoplanarity(px, normal_angle, plane_distance);
      
      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = 1;
      r.id_0 = p0->GetNodeID();
      r.id_1 = px->GetNodeID();
      r.rel_value.push_back(proximity);
      r.rel_value.push_back(colorSimilarity);
      r.rel_value.push_back(normal_angle);
      r.rel_value.push_back(plane_distance);
      r.remove = false;
      relations.push_back(r);
    }
  }
  
  /// calculate patch-line relations
  positive.resize(0);
  negative.resize(1);
  unsigned nrLines = kcore->NumGestalts3D(Gestalt3D::LINE);
  
    for(unsigned j=0; j<nrLines; j++)
    {
      Line3D *l = (Line3D*) kcore->Gestalts3D(Gestalt3D::LINE, j);

  for(unsigned i=0; i<nrPatches; i++)
  {
    Patch3D *p = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
      
      double proximity = PLProximity(p, l);
      double parallelity = PLParallelity(p, l);      

      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = 2;
      r.id_0 = p->GetNodeID();
      r.id_1 = l->GetNodeID();
      r.rel_value.push_back(proximity);
      r.rel_value.push_back(parallelity);
      r.remove = false;
      relations.push_back(r);
    }
  }
  rel = relations;
}

/**
 * @brief Print the result statistics.
 */
void CalculateRelations::PrintResults()
{
  printf("Results of relation calculation:\n");
  printf("  Number of relations: %u\n", relations.size());
  printf("  Accuracy: %4.3f\n", CheckAccuracy());
}

/**
 * @brief Print the result statistics.
 */
void CalculateRelations::PrintRelations()
{
  printf("Results of relation calculation:\n");
  printf("  Number of relations: %u\n", relations.size());
  printf("  Accuracy: %4.3f\n", CheckAccuracy());
  for(unsigned i=0; i< relations.size(); i++)
  {
    if(relations[i].rel_probability.size() >= 1) // check, if we have at least 2 labes (true/false)
    {
      if(relations[i].prediction == relations[i].groundTruth)
        printf("  Rel: %u: %u-%u => gt(%u) - pr(%u) (true) with prob: %4.3f\n", i, relations[i].id_0, relations[i].id_1, 
            relations[i].groundTruth, relations[i].prediction, relations[i].rel_probability[1]);
      else
        printf("  Rel: %u: %u-%u => gt(%u) - pr(%u) (false) with prob: %4.3f\n", i, relations[i].id_0, relations[i].id_1, 
            relations[i].groundTruth, relations[i].prediction, relations[i].rel_probability[1]);
    }
  }
}

/**
 * @brief Check the accuracy of the prediction, if values are available
 * @return Accuracy of prediction.
 */
double CalculateRelations::CheckAccuracy()
{
  unsigned truePredict = 0, falsePredict = 0, unknown = 0;
  for(unsigned i=0; i<relations.size(); i++)
  {
    if(relations[i].groundTruth == relations[i].prediction)
      truePredict++;
    else if (relations[i].groundTruth != -1)
      falsePredict++;
    else
      unknown++;
  }  
  
printf("  CalculateRelations::CheckAccuracy: Unknown labels: %u\n", unknown);
  
  if(relations.size() > 0)
    return ((double)truePredict / ((double) (truePredict + falsePredict)));
  else
    return 0.0;
}


/**
 * @brief Check, if line is inside patch ROI, after 3D->2D projection.
 * @param p Patch3D
 * @param l Line3D
 * @return Returns true, if line is in patch ROI.
 */
bool CalculateRelations::PLLineInPlaneROI(Patch3D *p, Line3D *l)
{
// Zwei Möglichkeiten:
// 1. Projektion in den Bild-Raum (patch + line) und überprüfen in 2D, oder
// 2. line-end-points auf die plane projezieren und dann sehen ob punkt innerhalb der convexen Hülle liegt

  bool startPoint = true, endPoint = true;

  // Project hull- and end-points
  std::vector<cv::Vec2f> hpr;
  std::vector<cv::Vec4f> hp = p->GetHullPoints();
  for(unsigned i=0; i<hp.size(); i++)
  {
    cv::Vec2f v;
    v[0] = cam_fx*hp[i][0]/hp[i][2] + cam_cx;
    v[1] = cam_fy*hp[i][1]/hp[i][2] + cam_cy;
    hpr.push_back(v);
  }
  cv::Vec2f s, e;
  s[0] = cam_fx*l->point[0][0]/l->point[0][2] + cam_cx;
  s[1] = cam_fy*l->point[0][1]/l->point[0][2] + cam_cy;
  e[0] = cam_fx*l->point[1][0]/l->point[1][2] + cam_cx;
  e[1] = cam_fy*l->point[1][1]/l->point[1][2] + cam_cy;
  
  int j;
  for(unsigned i=0; i<hpr.size(); i++)
  {
    j=i+1; 
    if(j == hpr.size()) j=0;
      
    cv::Vec2f vh;
    vh[0] =  hpr[j][0] - hpr[i][0];
    vh[1] =  hpr[j][1] - hpr[i][1];
    cv::Vec2f vhps;
    vhps[0] = s[0] - hpr[i][0];
    vhps[1] = s[1] - hpr[i][1];
    cv::Vec2f vhpe;
    vhpe[0] = e[0] - hpr[i][0];
    vhpe[1] = e[1] - hpr[i][1];

    // 2-dimensional cross product (z=0)
    double zs = vh[0]*vhps[1] - vh[1]*vhps[0];
    double ze = vh[0]*vhpe[1] - vh[1]*vhpe[0];
     
    if(zs < 0) startPoint = false;
    if(ze < 0) endPoint = false;

//     if(l->GetNodeID() == 910 || l->GetNodeID() == 940 || l->GetNodeID() == 970 || l->GetNodeID() == 1000 || l->GetNodeID() == 820)
//     {
//       printf("zs: %4.3f\n", zs);
//       printf("ze: %4.3f\n", ze);
//     }
  }
  
  if(!startPoint && !endPoint) return false;
  else return true;
  

/// 2.
  /// Diese Überprüfung checkt nur, ob der projezierte Punkt innerhalb der convexen Hülle
  /// liegt und nicht, ob eine Projektion mit dem Sichtstrahl innerhalb der convexen Hülle ist.
//   bool startPoint = true, endPoint = true;
//   double d0 = p->CalculatePointDistance(l->point[0][0], l->point[0][1], l->point[0][2]);
//   double d1 = p->CalculatePointDistance(l->point[1][0], l->point[1][1], l->point[1][2]);
//   cv::Vec3f n = p->GetPlaneNormal();
//   cv::Vec3f pr0;
//   pr0[0] = l->point[0][0] + n[0]*d0;
//   pr0[1] = l->point[0][1] + n[1]*d0;
//   pr0[2] = l->point[0][2] + n[2]*d0;
//   cv::Vec3f pr1;
//   pr1[0] = l->point[1][0] + n[0]*d1;
//   pr1[1] = l->point[1][1] + n[1]*d1;
//   pr1[2] = l->point[1][2] + n[2]*d1;
//   
//   int j;
//   std::vector<cv::Vec4f> hp = p->GetHullPoints();
//   for(unsigned i=0; i<hp.size(); i++)
//   {
//     j=i+1; 
//     if(j == hp.size()) j=0;
//       
//     cv::Vec3f vh;
//     vh[0] =  hp[j][0] - hp[i][0];
//     vh[1] =  hp[j][1] - hp[i][1];
//     vh[2] =  hp[j][2] - hp[i][2];
//     cv::Vec3f vhp0;
//     vhp0[0] = pr0[0] - hp[i][0];
//     vhp0[1] = pr0[1] - hp[i][1];
//     vhp0[2] = pr0[2] - hp[i][2];
//     cv::Vec3f vhp1;
//     vhp1[0] = pr1[0] - hp[i][0];
//     vhp1[1] = pr1[1] - hp[i][1];
//     vhp1[2] = pr1[2] - hp[i][2];
// 
//     // xxx = hp cross ph
//     cv::Vec3f cr0 = vh.cross(vhp0);
//     cv::Vec3f cr1 = vh.cross(vhp1);
//      
//     // xxx dot n < 0 ???
//     double crn0 = cr0.ddot(n);
//     double crn1 = cr1.ddot(n);
//     if(crn0 > 0) startPoint = false;
//     if(crn1 > 0) endPoint = false;
// 
// //     if(l->GetNodeID() == 910 || l->GetNodeID() == 940 || l->GetNodeID() == 970 || l->GetNodeID() == 1000 || l->GetNodeID() == 820)
// //     {
// //       printf("crn0: %4.3f\n", crn0);
// //       printf("crn1: %4.3f\n", crn1);
// //     }
//   }
//   
//   if(!startPoint && !endPoint) return false;
//   else return true;
}


/**
 * @brief Calculate proximity between patch and line.
 * @param p 3D-patch
 * @param l 3D-line
 */
double CalculateRelations::PLProximity(Patch3D *p, Line3D *l)
{
  double dist0 = p->CalculatePointDistance(l->point[0][0], l->point[0][1], l->point[0][2]);
  double dist1 = p->CalculatePointDistance(l->point[1][0], l->point[1][1], l->point[1][2]);
  if(dist0 < dist1) return dist0;
  else return dist1;
//   if(dist0 < 0.1 || dist1 < 0.1)
//     printf("%u-%u: Distance of line to patch: %4.3f / %4.3f\n", p->GetNodeID(), l->GetNodeID(), dist0, dist1);
}

/**
 * @brief Calculate proximity between patch and line.
 * @param p 3D-patch
 * @param l 3D-line
 */
double CalculateRelations::PLParallelity(Patch3D *p, Line3D *l)
{
  // Zwischen plane-normal und lineDirecton sollten 90° sein
  cv::Point3f dir = l->GetDirection();
  double para = p->CalculateParallelity(dir);
if(para != para) printf("CalculateRelations::PLParallelity: dir: %4.3f-%4.3f-%4.3f\n", dir.x, dir.y, dir.z);
  return para;
}

/**
 * @brief Constrain Relations after prediction
 */
void CalculateRelations::ConstrainRelations()
{
  /// Patch-Line: Each line has only one relation with a patch (with the highest probability)
  std::vector<Relation>::iterator it;
  for(unsigned i=0; i<relations.size(); i++)
  {
    for(unsigned j=i+1; j<relations.size(); j++)
    {
      if(relations[i].type == 2 && relations[j].type == 2 &&
         relations[i].id_1 == relations[j].id_1 &&
         !relations[i].remove && !relations[j].remove)
      {
          if(relations[i].rel_probability[1] > relations[j].rel_probability[1])
            relations[j].remove = true;
          else 
            relations[i].remove = true;
      }
    }
  }
  for(it=relations.end(); it>=relations.begin(); it--)
    if(it->remove)
      relations.erase(it);
}


}






















