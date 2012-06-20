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
{
  Reset();
}

/**
 * @brief Reset the relations
 */
void CalculateRelations::Reset()
{
  relations.resize(0);
  pxlsToDraw.clear();
}

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
}


/**
 * @brief Calculate relations using the ground-truth data to get positive and
 * negative examples for the SVM training.
 * @param rel Relations vector
 */
void CalculateRelations::CalcSVMRelations(std::vector<Relation> &rel)
{
  firstCall = true;
static struct timespec start, last, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
last = start;
//   relations.resize(0);
  CalcSVMPatchPatchRelations();
//   CalcSVMPatchLineRelations();
//   CalcSVMLineLineRelations();
  rel = relations;
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for CalculateRelations::CalcSVMRelations: %4.3f\n", timespec_diff(&current, &last));
}


/**
 * @brief Calculate relations between 3D-patches using the ground-truth data, 
 * to get positive and negative examples for the SVM training.
 */
void CalculateRelations::CalcSVMPatchPatchRelations()
{
printf("CalculateRelations::CalcSVMPatchPatchRelations: start!\n");
  unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(unsigned i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
// printf("  Calculate PP-Relation: %u-%u (%u-%u)\n", p0->GetNodeID(), px->GetNodeID(), p0->GetObjectLabel(), px->GetObjectLabel());
      
      // check if it belongs to the same object (ground truth) => True ground truth
      bool valid_relation = true;
      if(p0->GetObjectLabel() != 0 && p0->GetObjectLabel() == px->GetObjectLabel())
      {
        // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double n0n1, ppn0, ppn1;
        p0->CalculateCoplanarity2(px, n0n1, ppn0, ppn1);
        
        std::vector<double> segRelations;                       /// TODO Nur mehr für mask
        if(!CalculateSegmentRelations(p0, px, segRelations))
          valid_relation = false;
//         std::vector<double> colRelations;
//         if(!CalculatePPColorRelation(p0, px, colRelations))
//           valid_relation = false;
        std::vector<double> ppRelations;
        if(!CalculatePPRelation(p0, px, ppRelations))
          valid_relation = false;
       
// printf(" PP True : %u-%u: %4.3f - %4.3f - %4.3f - %4.3f - %4.3f\n", p0->GetNodeID(), px->GetNodeID(), proximity, colorSimilarity, n0n1, ppn0, ppn1);
// printf("  PP TRUE:\n");
        if(valid_relation)
        {
// printf(" ==> valid\n");
          Relation r;
          r.groundTruth = 1;
          r.prediction = -1;
          r.type = 1;
          r.id_0 = p0->GetNodeID();
          r.id_1 = px->GetNodeID();
          r.rel_value.push_back(proximity);           // proximity of planes (minimum hull distance)
          r.rel_value.push_back(colorSimilarity);     // color similarity (histogram) of the patch 
          r.rel_value.push_back(n0n1);                // coplanarity: angle between patch-normals
//           r.rel_value.push_back(ppn0);                // angle between center point line and first patch normal
//           r.rel_value.push_back(ppn1);                // angle between center point line and second patch normal
//           r.rel_value.push_back(segRelations[0]);     // distance to first plane
//           r.rel_value.push_back(segRelations[1]);     // distance to second plane
//           r.rel_value.push_back(segRelations[2]);     // depth value
          r.rel_value.push_back(segRelations[3]);     // mask value
//           r.rel_value.push_back(segRelations[4]);     // curvature value
          r.rel_value.push_back(ppRelations[0]);     // color similarity between the plane patches (pixel-wise) 
          r.rel_value.push_back(ppRelations[1]);     // depth value between neighboring plane pixels
          r.rel_value.push_back(ppRelations[3]);     // curvature value between neighboring plane pixels
          relations.push_back(r);
        }
      }
      else if((p0->GetObjectLabel() != 0 || px->GetObjectLabel() != 0) &&   // => False ground truth
               p0->GetObjectLabel() != px->GetObjectLabel())
      {
        // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double n0n1, ppn0, ppn1;
        p0->CalculateCoplanarity2(px, n0n1, ppn0, ppn1);
        
        std::vector<double> segRelations;                       /// TODO Nur mehr für mask
        if(!CalculateSegmentRelations(p0, px, segRelations))
          valid_relation = false;
//         std::vector<double> colRelations;
//         if(!CalculatePPColorRelation(p0, px, colRelations))
//           valid_relation = false;
        std::vector<double> ppRelations;
        if(!CalculatePPRelation(p0, px, ppRelations))
          valid_relation = false;

// printf("  PP FALSE:\n");

// printf(" PP False: %u-%u: %4.3f - %4.3f - %4.3f - %4.3f - %4.3f\n", p0->GetNodeID(), px->GetNodeID(), proximity, colorSimilarity, n0n1, ppn0, ppn1);
        if(valid_relation)
        {
// printf("  ==> valid\n");
          Relation r;
          r.groundTruth = 0;
          r.prediction = -1;
          r.type = 1;
          r.id_0 = p0->GetNodeID();
          r.id_1 = px->GetNodeID();
          r.rel_value.push_back(proximity);           // proximity of planes (minimum hull distance)
          r.rel_value.push_back(colorSimilarity);     // color similarity (histogram) of the patch 
          r.rel_value.push_back(n0n1);                // coplanarity: angle between patch-normals
//           r.rel_value.push_back(ppn0);                // angle between center point line and first patch normal
//           r.rel_value.push_back(ppn1);                // angle between center point line and second patch normal
//           r.rel_value.push_back(segRelations[0]);     // distance to first plane
//           r.rel_value.push_back(segRelations[1]);     // distance to second plane
//           r.rel_value.push_back(segRelations[2]);     // depth value
          r.rel_value.push_back(segRelations[3]);     // mask value
//           r.rel_value.push_back(segRelations[4]);     // curvature value
          r.rel_value.push_back(ppRelations[0]);     // color similarity between the plane patches (pixel-wise) 
          r.rel_value.push_back(ppRelations[1]);     // depth value between neighboring plane pixels
          r.rel_value.push_back(ppRelations[3]);     // curvature value between neighboring plane pixels
          relations.push_back(r);
        }
      }
    }
  }
// printf("CalculateRelations::CalcPatchPatchRelations: allRelations: %u\n", allRelations.size());
//   rel = ppRelations;
printf("CalculateRelations::CalcSVMPatchPatchRelations: end!\n");

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
  firstCall = true;
  
// printf("CalculateRelations::CalcTestRelations: Start\n");
  /// Patch-Patch relations
  int nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
  for(int i=0; i<nrPatches-1; i++)
  {
    Patch3D *p0 = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
    for(unsigned j=i+1; j<nrPatches; j++)
    {
      Patch3D *px = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, j);
      
      bool valid_relation = true;
      if(p0->GetNodeID() != -1 && px->GetNodeID() != -1)
      {
        // calculate relation values
        double proximity = p0->CalculateProximity(px);
        double colorSimilarity = p0->CompareColor(px);
        double n0n1, ppn0, ppn1;
        p0->CalculateCoplanarity2(px, n0n1, ppn0, ppn1);        
        
        std::vector<double> segRelations;                       /// TODO Nur mehr für mask
        if(!CalculateSegmentRelations(p0, px, segRelations))
          valid_relation = false;
//         std::vector<double> colRelations;
//         if(!CalculatePPColorRelation(p0, px, colRelations))
//           valid_relation = false;
        std::vector<double> ppRelations;
        if(!CalculatePPRelation(p0, px, ppRelations))
          valid_relation = false;
        
        if(valid_relation)
        {
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
          r.rel_value.push_back(proximity);           // proximity of planes (minimum hull distance)
          r.rel_value.push_back(colorSimilarity);     // color similarity (histogram) of the patch 
          r.rel_value.push_back(n0n1);                // coplanarity: angle between patch-normals
//           r.rel_value.push_back(ppn0);                // angle between center point line and first patch normal
//           r.rel_value.push_back(ppn1);                // angle between center point line and second patch normal
//           r.rel_value.push_back(segRelations[0]);     // distance to first plane
//           r.rel_value.push_back(segRelations[1]);     // distance to second plane
//           r.rel_value.push_back(segRelations[2]);     // depth value
          r.rel_value.push_back(segRelations[3]);     // mask value
//           r.rel_value.push_back(segRelations[4]);     // curvature value
          r.rel_value.push_back(ppRelations[0]);     // color similarity between the plane patches (pixel-wise) 
          r.rel_value.push_back(ppRelations[1]);     // depth value between neighboring plane pixels
          r.rel_value.push_back(ppRelations[3]);     // curvature value between neighboring plane pixels
          relations.push_back(r);
        }
      }
    }
  }
// printf("CalculateRelations::CalcTestRelations: Patch-Patch calculated!\n");
  
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
// printf("CalculateRelations::CalcTestRelations: Patch-Line calculated!\n");

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
  firstCall = true;

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
//       double normal_angle, plane_distance; 
//       p0->CalculateCoplanarity(px, normal_angle, plane_distance);
      double n0n1, ppn0, ppn1;
      p0->CalculateCoplanarity2(px, n0n1, ppn0, ppn1);        
      
      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = 1;
      r.id_0 = p0->GetNodeID();
      r.id_1 = px->GetNodeID();
      r.rel_value.push_back(proximity);
      r.rel_value.push_back(colorSimilarity);
      r.rel_value.push_back(n0n1);
      r.rel_value.push_back(ppn0);
      r.rel_value.push_back(ppn1);
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
  PrintResults();
  for(unsigned i=0; i< relations.size(); i++)            /// TODO TODO Überarbeiten!!!
  {
    if(relations[i].rel_probability.size() >= 1) // check, if we have at least 2 labes (true/false)
    {
      if(relations[i].prediction == relations[i].groundTruth)
      {
        printf("  Rel: %u: %u-%u => gt(%u) - pr(%u) (true) with prob: %4.3f\n", i, relations[i].id_0, relations[i].id_1, 
            relations[i].groundTruth, relations[i].prediction, relations[i].rel_probability[1]);
      }
      else
      {
        printf("  Rel: %u: %u-%u => gt(%u) - pr(%u) (false) with prob: %4.3f\n", i, relations[i].id_0, relations[i].id_1, 
            relations[i].groundTruth, relations[i].prediction, relations[i].rel_probability[1]);
      }
      printf("    ");
      for(unsigned k=0; k< relations[i].rel_value.size(); k++)
      {
        printf("%4.3f - ", relations[i].rel_value[k]);
      }
      printf("\n");
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
if(para != para) printf("CalculateRelations::PLParallelity: Warning: dir: %4.3f-%4.3f-%4.3f\n", dir.x, dir.y, dir.z);
  return para;
}

/**
 * @brief Constrain relations after prediction.
 * Patch-Line: Only the line with the highest probability is assigned to a patch.
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

/**
 * @brief Calculates the minimum distance between given image index-points
 * and given coordinates on the image plane.
 * @param indexes Vector of image indexes
 * @param x x-coordinate
 * @param y y-coordinate
 * @param width image width
 * @param height image height
 * @param u x-coordinate of minimum distance point
 * @param v y-coordinate of minimum distance point
 * @return Returns the minimum distance between the indexes and the given coordinates.
 */
double IndexDistance(std::vector<int> indexes,
                     double x, double y,
                     int width, int height,
                     double &u, double &v)
{
  u=0.; v=0.;
  double a, b;
  double dx, dy;
  double distance, min_dist = (double) (width+height);
  for(unsigned i=0; i<indexes.size(); i++)
  {
    a = indexes[i] % width;
    b = indexes[i] / width;
    dx = fabs((double) a-x);
    dy = fabs((double) b-y);
    distance = sqrt(dx*dx + dy*dy);
    if(distance < min_dist)
    {
      min_dist = distance;
      u = a;
      v = b;
    }
  }
  return min_dist;
}

/**
 * @brief Calculate the relations between patches, caused by segments.
 * @param p0 First patch
 * @param p1 Second patch
 * @param params Parameter vector:
 *  - distance p0
 *  - distance p1
 *  - depth value
 *  - mask value
 *  - curvature value
 * @return Returns true, if there is a relation between these patches.
 */
bool CalculateRelations::CalculateSegmentRelations(Patch3D *p0, Patch3D *p1, std::vector<double> &params)
{
  /** Distance threshold between segments and patches **/
  static double MIN_DISTANCE_THR = 5.;

  /** Use weighting with edgel distances **/
  static bool USE_DISTANCE_WEIGHTING = false;
  
  /** Use normalisation of results from edgel length and number of edges **/
  static bool USE_LENGTH_NORMALISATION = true;
  
  int width = kcore->GetPointCloudWidth();
  int height = kcore->GetPointCloudHeight();
  
  unsigned nr_edges = 0;
  unsigned overall_nr_points = 0;
  double pp_dis_sum_0 = 0;
  double pp_dis_sum_1 = 0;
  double pp_depth_sum = 0;
  double pp_mask_sum = 0;
  double pp_curv_sum = 0;
  
  // First calculate the distances of the patch-hulls to the segment-edgels and store it.
  if(firstCall)
  {
    min_distances.clear();
    distances.clear();
    unsigned nrPatches = kcore->NumGestalts3D(Gestalt3D::PATCH);
    for(unsigned i=0; i<nrPatches; i++)
    {
      std::vector<double> min_patch_distances;
      std::vector< std::vector<double> > patch_distances;
      Patch3D *patch = (Patch3D*) kcore->Gestalts3D(Gestalt3D::PATCH, i);
      unsigned nrSegments = kcore->NumGestalts3D(Gestalt3D::SEGMENT);
      for(unsigned j=0; j<nrSegments; j++)
      {
        Segment3D *s = (Segment3D*) kcore->Gestalts3D(Gestalt3D::SEGMENT, j);

        std::vector<int> indexes_p, indexes_s;
        double x, y, dis, min_dist, a, b;

        patch->GetIndexes(indexes_p);
        s->GetIndexes(indexes_s);
        x = indexes_s[0] % width; // get coordinate of segment start point
        y = indexes_s[0] / width;
        min_dist = IndexDistance(indexes_p, x, y, width, height, a, b);  // min distance between patch and segment start point
        dis = min_dist - indexes_s.size();  // Distance is minimum distance of start_point - length of segment 
                                            // => maybe negative => good idea to constrain calculations?
        if(dis < MIN_DISTANCE_THR)  // is valid, if startPoint-segmentLength < MIN_DISTANCE_THR
        {        
          double abs_min_dist = (double) width;
          std::vector<double> p_distances;
          for(unsigned k=0; k<indexes_s.size(); k++)
          {
            x = indexes_s[k] % width;
            y = indexes_s[k] / width;
            min_dist = IndexDistance(indexes_p, x, y, width, height, a, b);
            if(min_dist < abs_min_dist)
              abs_min_dist = min_dist;
            p_distances.push_back(min_dist);
          }
          min_patch_distances.push_back(abs_min_dist);
          patch_distances.push_back(p_distances);
        }
        else
        {
          min_patch_distances.push_back(dis);
          std::vector<double> dist;
          patch_distances.push_back(dist);      // Empty vector!!!
        }
      }
      min_distances.push_back(min_patch_distances);
      distances.push_back(patch_distances);
    }
    firstCall = false;
  }
  
  // find segments next to the two patches
  unsigned nrSegments = kcore->NumGestalts3D(Gestalt3D::SEGMENT);
  for(unsigned i=0; i<nrSegments; i++)
  {
    if(min_distances[p0->GetNodeID()][i] < MIN_DISTANCE_THR && min_distances[p1->GetNodeID()][i] < MIN_DISTANCE_THR)
    {
      nr_edges++;
      Segment3D *s = (Segment3D*) kcore->Gestalts3D(Gestalt3D::SEGMENT, i);

      int nr_points = 0;
      double dis_sum_0 = 0.;
      double dis_sum_1 = 0.;
      double depth_sum = 0.;
      double mask_sum = 0.;
      double curv_sum = 0.;
      
      std::vector<int> indexes_s;
      s->GetIndexes(indexes_s);
      for(unsigned j=0; j<indexes_s.size(); j++)
      {
        double dis_0 = distances[p0->GetNodeID()][i][j];  // distance of segment edgel to patch p0
        double dis_1 = distances[p1->GetNodeID()][i][j];  // distance of segment edgel to patch p1
        if(dis_0 < MIN_DISTANCE_THR && dis_1 < MIN_DISTANCE_THR)
        {
          nr_points++;
          overall_nr_points++;
          dis_sum_0 += dis_0;
          dis_sum_1 += dis_1;
          if(USE_DISTANCE_WEIGHTING)
          {
            dis_0 = (MIN_DISTANCE_THR - dis_0) / MIN_DISTANCE_THR;  // normalization (0-1)
            dis_1 = (MIN_DISTANCE_THR - dis_1) / MIN_DISTANCE_THR;
            depth_sum += (s->GetEdgeSupportDepth(j) *dis_0*dis_1);
            mask_sum += (s->GetEdgeSupportMask(j) *dis_0*dis_1);
            curv_sum += (s->GetEdgeSupportCurvature(j) *dis_0*dis_1);   
          }
          else
          {
            depth_sum += s->GetEdgeSupportDepth(j);
            mask_sum += s->GetEdgeSupportMask(j);
            curv_sum += s->GetEdgeSupportCurvature(j);   
          }
        }
      }
      
      if(nr_points != 0)
      {
        if(USE_LENGTH_NORMALISATION)
        {
          dis_sum_0 /= nr_points;                 /// TODO Sollte man das Normieren????
          dis_sum_1 /= nr_points;                 /// Wenn ein edgel zwei Flächen gut teilt, dann ist doch egal, wo die anderen edgels sind, oder?
          depth_sum /= nr_points;                 /// 1. Zum Beispiel eine Linie, die dann in eine Schattenlinie übergeht würde schlechtere Werte liefern als
          mask_sum /= nr_points;                  ///    eine Linie die nur direkt an der Kante ist, obwohl sie das gleiche aussagen. => Eigentlich werden weit entfernte pixel nicht gezählt (DIST_THR0)
          curv_sum /= nr_points;                  /// 2. Eine längere Linie zwischen 2 Flächen hat mehr aussagekraft als nur 2 Pixel
        }
        
        pp_dis_sum_0 += dis_sum_0;
        pp_dis_sum_1 += dis_sum_1;
        pp_depth_sum += depth_sum;
        pp_mask_sum += mask_sum;
        pp_curv_sum += curv_sum;
      }
    }
  }

  if(nr_edges != 0)
  {  
    if(USE_LENGTH_NORMALISATION)
    {
      pp_dis_sum_0 /= nr_edges;
      pp_dis_sum_1 /= nr_edges;
      pp_depth_sum /= nr_edges;
      pp_mask_sum /= nr_edges;
      pp_curv_sum /= nr_edges;
    }
//printf("Solution: %u-%u (%u pts) (dis: %4.3f/%4.3f) => dep: %4.3f - mask: %4.3f - curv: %4.3f\n", p0->GetNodeID(), p1->GetNodeID(), overall_nr_points, 
//         pp_dis_sum_0, pp_dis_sum_1, pp_depth_sum, pp_mask_sum, pp_curv_sum);
    params.push_back(pp_dis_sum_0);
    params.push_back(pp_dis_sum_1);
    params.push_back(pp_depth_sum);
    params.push_back(pp_mask_sum);
    params.push_back(pp_curv_sum);
    return true;
  }
  else
  {
    params.push_back(/*MIN_DISTANCE_THR*/width);
    params.push_back(/*MIN_DISTANCE_THR*/width);
    params.push_back(0.);
    params.push_back(0.);
    params.push_back(0.);
    return false;
  }
}


/**
 * @brief Calculate color relation between neighboring patches.
 * Calculate value for color similarity between neighboring patches.
 * We can not use the segment calculation, because, if there is no color edge, we do not have a segment!
 * @param p0 First patch
 * @param p1 Second patch
 * @param params Parameter vector:
 * - Value for color similarity
 */
bool CalculateRelations::CalculatePPColorRelation(Patch3D *p0, Patch3D *p1, std::vector<double> &params)
{
printf("CalculateRelations::CalculatePPColorRelation: Warning: Antiquated!\n");
  
  /** Distance threshold between segments and patches **/
  static double MIN_DISTANCE_THR_PP_COLOR = 3.;
  static double MIN_DISTANCE_THR_PP_DEPTH = 1.5;

  /** Use weighting with edgel distances **/
//   static bool USE_DISTANCE_WEIGHTING_PP = false;
  
  /** Use normalisation of results from edgel length and number of edges **/
//   static bool USE_LENGTH_NORMALISATION_PP = true;

  
// printf("CalculatePPColorRelation: start!\n");
  /// TODO Schnellerer Ansatz => Das Selbe gilt für CalculateSegmentRelations
  // Nehme beide Patch-Masken, dann
  // - erweitern der mask-edges um 2 pixel
  // - lege sie übereinander und suche nach Übereinstimmung
  // Wenn gefunden, dann sind die Patches "nebeneinander:
  // - Berechne für jeden Pixel den nähesten Pixel des anderen Patches
  // - summiere die Farbunerschiede auf, wenn Pixel nahe beieinander sind.
  
  
  /// Brute-Force Ansatz (alles mit allem vergleichen!)
  // Get indexes
  // - Berechne für jeden Punkt den nähesten anderen Pixel und berechne den Farbunterschied!

  int width = kcore->GetPointCloudWidth();
  int height = kcore->GetPointCloudHeight();
  
  std::vector<int> indexes_p0, indexes_p1;
  p0->GetIndexes(indexes_p0);
  p1->GetIndexes(indexes_p1);

  double min_dist = width;
  double u, v;
  double nr_valid_points = 0;
  double uv_color_distance = 0.0;
  double uv_color_distance_dist = 0.0;
  
  double depth_sum = 0.;
  
  /// find for every plane-mask-edge-point the nearest neighbor to the other plane
  for(unsigned i=0; i<indexes_p0.size(); i++)
  {
    double x, y;
    x = indexes_p0[i] % width;
    y = indexes_p0[i] / width;
    min_dist = IndexDistance(indexes_p1, x, y, width, height, u, v);

    /// Now get the color of x,y and u,v
    int p0_idx = (int) (y*width + x);
    int p1_idx = (int) (v*width + u);
    RGBValue p0_color, p1_color;

    if(min_dist < MIN_DISTANCE_THR_PP_COLOR)
    {          
      nr_valid_points++;
            
      /// compare color
      p0_color.float_value = kcore->GetPclCloud()->points[p0_idx].rgb;
      p1_color.float_value = kcore->GetPclCloud()->points[p1_idx].rgb;

//       double p0_Y =  (0.257 * p0_color.r) + (0.504 * p0_color.g) + (0.098 * p0_color.b) + 16;
      double p0_U = -(0.148 * p0_color.r) - (0.291 * p0_color.g) + (0.439 * p0_color.b) + 128;
      double p0_V =  (0.439 * p0_color.r) - (0.368 * p0_color.g) - (0.071 * p0_color.b) + 128;
//       double p1_Y =  (0.257 * p1_color.r) + (0.504 * p1_color.g) + (0.098 * p1_color.b) + 16;
      double p1_U = -(0.148 * p1_color.r) - (0.291 * p1_color.g) + (0.439 * p1_color.b) + 128;
      double p1_V =  (0.439 * p1_color.r) - (0.368 * p1_color.g) - (0.071 * p1_color.b) + 128;
      
//       double y_1 = p0_Y/255 - p1_Y/255;
//       double y_2 = y_1 * y_1 * 100;
      double u_1 = p0_U/255 - p1_U/255;
      double u_2 = u_1 * u_1 * 100;
      double v_1 = p0_V/255 - p1_V/255;
      double v_2 = v_1 * v_1 * 100;
      double cDist = sqrt(u_2 + v_2);
//           double cDist = sqrt((p0_color.r - p1_color.r)*(p0_color.r - p1_color.r) + (p0_color.g - p1_color.g)*(p0_color.g - p1_color.g) + (p0_color.b - p1_color.b)*(p0_color.b - p1_color.b));
// printf("  cDist: %2.2f-%2.2f-%2.2f =>  %4.3f (with min_dist: %4.3f and ", y_1, u_1, v_1, cDist, min_dist);
      double cDist_dist = cDist*(min_dist)/MIN_DISTANCE_THR_PP_COLOR;
// printf("norm: %4.3f\n",  cDist_dist);
      uv_color_distance += cDist;                         // TODO Säubern!
      uv_color_distance_dist += cDist_dist;
    }
//         else printf("At least found, but to far away?\n");

    if(min_dist < MIN_DISTANCE_THR_PP_DEPTH)
    {    
      /// compare depth!
//       cv::Vec4f pt;
//       pt[0]= kcore->GetPclCloud()->points[p0_idx].x;
//       pt[1]= kcore->GetPclCloud()->points[p0_idx].y;
//       pt[2]= kcore->GetPclCloud()->points[p0_idx].z;
//       pt[3]= kcore->GetPclCloud()->points[p0_idx].rgb;
// pxlsToDraw.push_back(pt);
//       pt[0]= kcore->GetPclCloud()->points[p1_idx].x;
//       pt[1]= kcore->GetPclCloud()->points[p1_idx].y;
//       pt[2]= kcore->GetPclCloud()->points[p1_idx].z;
//       pt[3]= kcore->GetPclCloud()->points[p1_idx].rgb;
// pxlsToDraw.push_back(pt);

      double p0_z = kcore->GetPclCloud()->points[p0_idx].z;
      double p1_z = kcore->GetPclCloud()->points[p1_idx].z;
      double depth = fabs(p0_z - p1_z);
      depth_sum += depth;
      
if(depth > 4.0) printf("  CalculateRelations: Warning: depth too large => d0-d1: %4.3f / %4.3f\n", p0_z, p1_z);
    }
  }
  
  /// normalize color values!
//   printf("  uv_sqr_distance: %4.3f\n",  uv_color_distance);
//   printf("  uv_sqr_distance_dist: %4.3f\n",  uv_color_distance_dist);
  if(nr_valid_points != 0.)
  {
    /// TODO add here normalisation flag!
    uv_color_distance /= nr_valid_points;
    uv_color_distance_dist /= nr_valid_points;
    depth_sum /= nr_valid_points;
    
// printf("CalculatePPColorRelation: %u / %u: ", p0->GetNodeID(), p1->GetNodeID());
//   printf("%4.3f - ",  uv_color_distance);
//   printf("normiert: %4.3f",  uv_color_distance_dist);
//   printf("  depth_sum: %4.3f\n\n",  depth_sum);

    params.push_back(uv_color_distance_dist);
    params.push_back(depth_sum);
    return true;
  }
//   else
//   {
// printf("CalculatePPColorRelation: %u-%u are not neighboring planes.\n", p0->GetNodeID(), p1->GetNodeID());
    return false;
//   }
}


/**
 * @brief Calculate color relation between neighboring patches.
 * Calculate value for color similarity between neighboring patches.
 * We can not use the segment calculation, because, if there is no color edge, we do not have a segment!
 * @param p0 First patch
 * @param p1 Second patch
 * @param params Parameter vector:
 * - Value for color similarity
 */
bool CalculateRelations::CalculatePPRelation(Patch3D *p0, Patch3D *p1, std::vector<double> &params)
{
  /** Distance threshold between segments and patches **/
  static double MIN_DISTANCE_THR_PP_COLOR = 1.5;
  static double MIN_DISTANCE_THR_PP_DEPTH = 1.5;
  static double MIN_DISTANCE_THR_PP_MASK = 1.5;
  static double MIN_DISTANCE_THR_PP_CURVATURE = 1.5;

  /** Use weighting with edgel distances **/
//   static bool USE_DISTANCE_WEIGHTING_PP = false;
  
  /** Use normalisation of results from edgel length and number of edges **/
  static bool USE_COLOR_NORMALISATION = true;
  static bool USE_DEPTH_NORMALISATION = true;
  static bool USE_MASK_NORMALISATION = true;
  static bool USE_CURVATURE_NORMALISATION = true;

  int width = kcore->GetPointCloudWidth();
  int height = kcore->GetPointCloudHeight();
  
  std::vector<int> indexes_p0, indexes_p1;
  p0->GetIndexes(indexes_p0);
  p1->GetIndexes(indexes_p1);

  double min_dist = width;                // minimum distance between plane hull points
  double uv_color_distance = 0.0;
  
  double nr_valid_points_color = 0;
  double nr_valid_points_depth = 0;
  double nr_valid_points_mask = 0;
  double nr_valid_points_curvature = 0;
  
  double color_sum = 0.0;
  double depth_sum = 0.0;
  double mask_sum = 0.0;
  double curvature_sum = 0.0;

  /// find for every plane-mask-edge-point the nearest neighbor to the other plane
  for(unsigned i=0; i<indexes_p0.size(); i++)
  {
    double x, y;
    double u, v;
    x = indexes_p0[i] % width;
    y = indexes_p0[i] / width;
    min_dist = IndexDistance(indexes_p1, x, y, width, height, u, v);
    int p0_idx = (int) (y*width + x); 
    int p1_idx = (int) (v*width + u);

    /// calculate color distance
    if(min_dist < MIN_DISTANCE_THR_PP_COLOR)
    {    
      nr_valid_points_color++;
      RGBValue p0_color, p1_color;
      p0_color.float_value = kcore->GetPclCloud()->points[p0_idx].rgb;
      p1_color.float_value = kcore->GetPclCloud()->points[p1_idx].rgb;

//       double p0_Y =  (0.257 * p0_color.r) + (0.504 * p0_color.g) + (0.098 * p0_color.b) + 16;
      double p0_U = -(0.148 * p0_color.r) - (0.291 * p0_color.g) + (0.439 * p0_color.b) + 128;
      double p0_V =  (0.439 * p0_color.r) - (0.368 * p0_color.g) - (0.071 * p0_color.b) + 128;
//       double p1_Y =  (0.257 * p1_color.r) + (0.504 * p1_color.g) + (0.098 * p1_color.b) + 16;
      double p1_U = -(0.148 * p1_color.r) - (0.291 * p1_color.g) + (0.439 * p1_color.b) + 128;
      double p1_V =  (0.439 * p1_color.r) - (0.368 * p1_color.g) - (0.071 * p1_color.b) + 128;
      
//       double y_1 = p0_Y/255 - p1_Y/255;
//       double y_2 = y_1 * y_1 * 100;
      double u_1 = p0_U/255 - p1_U/255;
      double u_2 = u_1 * u_1 * 100;
      double v_1 = p0_V/255 - p1_V/255;
      double v_2 = v_1 * v_1 * 100;
      double cDist = sqrt(u_2 + v_2);
//           double cDist = sqrt((p0_color.r - p1_color.r)*(p0_color.r - p1_color.r) + (p0_color.g - p1_color.g)*(p0_color.g - p1_color.g) + (p0_color.b - p1_color.b)*(p0_color.b - p1_color.b));
// printf("  cDist: %2.2f-%2.2f-%2.2f =>  %4.3f (with min_dist: %4.3f and ", y_1, u_1, v_1, cDist, min_dist);
      double cDist_dist = cDist*(min_dist)/MIN_DISTANCE_THR_PP_COLOR;
// printf("norm: %4.3f\n",  cDist_dist);
      uv_color_distance += cDist;                         // TODO Säubern!
      color_sum += cDist_dist;
    }
//         else printf("At least found, but to far away?\n");

    /// calculate depth
    if(min_dist < MIN_DISTANCE_THR_PP_DEPTH)
    {    
      nr_valid_points_depth++;

      double p0_z = kcore->GetPclCloud()->points[p0_idx].z;
      double p1_z = kcore->GetPclCloud()->points[p1_idx].z;
      double depth = fabs(p0_z - p1_z);
      depth_sum += depth;
      if(depth > 4.0) printf("CalculateRelations::CalculatePPRelation: Warning: depth too large => d0-d1: %4.3f / %4.3f\n", p0_z, p1_z);
    }
  
    /// calculate mask (TODO How???)
//     if(min_dist < MIN_DISTANCE_THR_PP_MASK)
//     {    
// //       nr_valid_points_mask++;
//     }  
  
    /// calculate curvature
    if(min_dist < MIN_DISTANCE_THR_PP_CURVATURE)
    {    
      nr_valid_points_curvature++;
      
      cv::Vec3f pt0, pt1;
      pt0[0]= kcore->GetConstPclCloud()->points[p0_idx].x;
      pt0[1]= kcore->GetConstPclCloud()->points[p0_idx].y;
      pt0[2]= kcore->GetConstPclCloud()->points[p0_idx].z;
      pt1[0]= kcore->GetConstPclCloud()->points[p1_idx].x;
      pt1[1]= kcore->GetConstPclCloud()->points[p1_idx].y;
      pt1[2]= kcore->GetConstPclCloud()->points[p1_idx].z;
      
      cv::Vec3f p0_normal = p0->GetPlaneNormal();
      cv::Vec3f p1_normal = p1->GetPlaneNormal();
      cv::Vec3f pp = pt1 - pt0;

      double norm_pp = cv::norm(pp);
      cv::Vec3f pp_dir;
      pp_dir[0] = pp[0]/norm_pp;
      pp_dir[1] = pp[1]/norm_pp;
      pp_dir[2] = pp[2]/norm_pp;  

// printf("norm_pp: %4.3f - ppdir: %4.3f-%4.3f-%4.3f\n", norm_pp, pp_dir[0], pp_dir[1], pp_dir[2]);
//       double dot0 = p0_normal.ddot(pp_dir);
      double a_p0_pp = acos(p0_normal.ddot(pp_dir));
      pp_dir = -pp_dir; // invert direction between points
      double a_p1_pp = acos(p1_normal.ddot(pp_dir));
      
      curvature_sum += a_p0_pp+a_p1_pp - M_PI;
    }
  }
  
  /// normalize values!
  if(nr_valid_points_color == 0. && nr_valid_points_depth == 0. &&
     nr_valid_points_mask == 0. && nr_valid_points_curvature == 0.)
    return false;

  if(nr_valid_points_color != 0. && USE_COLOR_NORMALISATION)
    color_sum /= nr_valid_points_color;
  if(nr_valid_points_depth != 0. && USE_DEPTH_NORMALISATION)
    depth_sum /= nr_valid_points_depth;
  if(nr_valid_points_mask != 0. && USE_MASK_NORMALISATION)
    mask_sum /= nr_valid_points_mask;
  if(nr_valid_points_curvature != 0. && USE_CURVATURE_NORMALISATION)
    curvature_sum /= nr_valid_points_curvature;
    
// printf("CalculatePPRelation: %u-%u: %4.3f - %4.3f - %4.3f - %4.3f\n", 
//        p0->GetNodeID(), p1->GetNodeID(), color_sum, depth_sum, mask_sum, curvature_sum);

  params.push_back(color_sum);
  params.push_back(depth_sum);
  params.push_back(mask_sum);
  params.push_back(curvature_sum);
  return true;
}


/**
 * @brief Debug thing.
 */
void CalculateRelations::GetPixelsToDraw(std::vector<cv::Vec4f> &pts)
{
  pts = pxlsToDraw;
}


}






















