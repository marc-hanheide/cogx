//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "multiplatform.hpp"

#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "syFormClosedSegmentGroups.hpp"
#include "syEllipse.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H
#include OCV_CXCORE_H
#include OCV_HIGHGUI_H


NAMESPACE_CLASS_BEGIN( RTE )

////////////////////////////////////////////////////////////////////////////////
CzFormClosedSegmentGroups::CzFormClosedSegmentGroups()
   : voteImg(0)
{
}

////////////////////////////////////////////////////////////////////////////////
CzFormClosedSegmentGroups::~CzFormClosedSegmentGroups()
{
   if (voteImg != 0) delete voteImg;
}


////////////////////////////////////////////////////////////////////////////////
// Release vote image
void CzFormClosedSegmentGroups::Reset(IplImage *img)
{
   if (voteImg != 0 &&
     !(voteImg->width == (int)img->width &&
         voteImg->height == (int)img->height))
   {
      delete voteImg;
      voteImg = 0;
   }
   if (voteImg == 0)
   {
      voteImg = new CzVoteImage(img->width, img->height);
      if (voteImg == 0)
         throw CzExcept(__HERE__, "Could not create instance of CzVoteImage! Out of memory?");
   }
   voteImg->Clear();
}

////////////////////////////////////////////////////////////////////////////////
// end means if the origin point is end or start of a segment
bool inline FollowPath(bool rightDir, CzSegment *curr, CzSegment *searched, bool end, int it) 
{
	if (curr == NULL) {
		return false;//path is over	
	}
	
	if (curr == searched) return true;

	if (curr->end == NULL || curr->start == NULL) {
		//cout << "Not able to finish path (All segments in a closed contour must be connected - both sides)" << endl;		
		return false;
	}

	if (it > 10) {
		//cout << "Not able to finish path" << endl;		
		return false;
	}

	if (rightDir) {
		if (end) {
			return FollowPath(curr->e, curr->end, searched, true, ++it);
		} else {
			return FollowPath(curr->s, curr->start, searched, false, ++it);
		} 
	} else {
		if (end) {
			return FollowPath(curr->s, curr->start, searched, false, ++it);
		} else {
			return FollowPath(curr->e, curr->end, searched, true, ++it);
		}
	}

	return false;
}

////////////////////////////////////////////////////////////////////////////////
// Check if contour is closed
bool inline ContourClosed(CzSegment *seg, bool checkPath) 
{
	//if(seg == NULL || seg->edgels[0] == NULL) return false;

	if( abs( seg->edgels[0].p.x - seg->edgels[seg->edgels.Size()-1].p.x ) <= 2 &&
		 abs( seg->edgels[0].p.y - seg->edgels[seg->edgels.Size()-1].p.y ) <= 2 ) {
			return true;	
	}	

	if(!checkPath) return false;
	if(seg->start == NULL || seg->end == NULL) return false; //segment must be connected on both sides in order to be closed!
	//check also those CzEllipses where start and end tangent may intersect with each other (only one connected but incomplete segment)
	//they wont as VoteImage checks this cases and ignores it (VoteImage modified)!
	if(seg->start == seg && seg->end == seg) return true; 
	//if seg->e, then follow seg->end->end, otherwise follow seg->end->start
	//start following path (we always choose the end part to start)
	return FollowPath(seg->e, seg->end, seg, true, 0);
	return false;
}

////////////////////////////////////////////////////////////////////////////////
// Close a contour
void CzFormClosedSegmentGroups::CloseContour(CzSegment *seg, CzArray<int> &idsUsed, CzSegment *joined) 
{
   CzArray<CzEdgel> arr = seg->edgels;
   CzArray<CzEdgel> arr1;		
   CzSegment *n;
   n = seg->end;
   idsUsed.PushBack(seg->id);
   bool rightDir = seg->e;
   bool end = true;
   unsigned it = 0;

   while (n != NULL && n != seg && it < 10) {
      idsUsed.PushBack(n->id);
      if (rightDir) {
         // add edgels in the same order
         arr1.Clear();
         arr1 = n->edgels;
         for (unsigned iii = 0; iii < arr1.Size(); iii++)
            arr.PushBack(arr1[iii]);
					
         if (end) {					
            rightDir = n->e;					
            n = n->end;
            end = true;
         } else {
            rightDir = n->s;
            n = n->start;
            end = false;
         }	
      } else  {
         // add edgels in reverse order
         arr1.Clear();					
         arr1 = n->edgels;
         for (int iii = arr1.Size() - 1; iii >= 0; iii--)
            arr.PushBack(arr1[iii]);
					
         if (end) {					
            rightDir = n->s;					
            n = n->start;
            end = false;
         } else {
            rightDir = n->e;
            n = n->end;
            end = true;
         }
      }
      it++;
   }

   joined->edgels = arr;
}

////////////////////////////////////////////////////////////////////////////////
// Find lines
void CzFormClosedSegmentGroups::Operate(CzArray<CzSegment*> &segments, int iLinkingGrowCount)
{
   if (voteImg==0)
      throw CzExcept(__HERE__,"You have to reset the vote image!");

   CzSegment *seg;
   unsigned sline;
   float params[4];
   unsigned idx;
   CzVector2 dir, dirP;
   CzVector2 p1,p2;
   double len=1000;
   if (iLinkingGrowCount < 0) {
      iLinkingGrowCount = 50;
   }
   unsigned z, numGrow = (unsigned)iLinkingGrowCount;
   CzArray<CzVoteImage::CzElem> iscts;
	
   voteImg->SetNumLines(segments.Size()*8);
	unsigned int min_points=12;
	unsigned int points_used = 12;
	unsigned int point_used=points_used-1;
	CvMat *points = cvCreateMat(1, points_used, CV_32FC2 );
   if (points == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	//IplImage *img = cvCreateImage(cvSize(voteImg->width,voteImg->height), IPL_DEPTH_8U, 3);

   for (unsigned i=0; i<segments.Size(); i++) {
      
      seg = segments[i];
      if (seg->edgels.Size()<min_points) 
         continue;
      
      sline = seg->id * 8;

		if (ContourClosed(seg, false)) {
			seg->closed = true;
			continue;
		}

      //draw lines (edge is drawn as TS and TE)
      for (unsigned j=0; j<(unsigned int)(seg->edgels.Size()/2); j++) {      
			voteImg->SetPixel(cvRound(seg->edgels[j].p.x), 
			                  cvRound(seg->edgels[j].p.y), 
			                  sline + VOTE_TS);
		}

		for (unsigned j=(unsigned int)(seg->edgels.Size()/2)+1; j<seg->edgels.Size(); j++) {      
			voteImg->SetPixel(cvRound(seg->edgels[j].p.x), 
			                  cvRound(seg->edgels[j].p.y), 
			                  sline + VOTE_TE);
		}

		//compute TS
		idx=0;
		for(unsigned j=0; j < points_used; j++) {
			points->data.fl[j*2] = (float) seg->edgels[points_used - 1 - idx].p.x;
			points->data.fl[j*2+1] = (float) seg->edgels[points_used - 1 - idx].p.y;
			idx++;
		}

      cvFitLine( points, CV_DIST_L2, 0, 0.01, 0.01, params );
      dir = CzVector2(params[0], params[1]);

		//check direction
		p1 = CzVector2(seg->edgels[point_used].p.x, seg->edgels[point_used].p.y);
		p2 = CzVector2(seg->edgels[0].p.x, seg->edgels[0].p.y);
		dirP = p2-p1;
		dirP.Normalise();
		if( (dir.x < 0 && dirP.x > 0) || (dir.x > 0 && dirP.x < 0)) dir.x = -dir.x;
		if( (dir.y < 0 && dirP.y > 0) || (dir.y > 0 && dirP.y < 0)) dir.y = -dir.y;

		voteImg->InitLine(cvRound(seg->edgels[points_used-1].p.x), cvRound(seg->edgels[points_used-1].p.y), cvRound(seg->edgels[points_used-1].p.x + dir.x*len), cvRound(seg->edgels[points_used-1].p.y + dir.y*len), sline + VOTE_TS);

		//compute TE
		idx=seg->edgels.Size();
		unsigned k=0;
		for(unsigned j=points_used; j > 0; j--, k++) {
			points->data.fl[k*2] = (float) seg->edgels[idx-j].p.x;
			points->data.fl[k*2+1] = (float) seg->edgels[idx-j].p.y;
		}
		
		idx=seg->edgels.Size()-1;
		cvFitLine( points, CV_DIST_L2, 0, 0.01, 0.01, params );
      dir = CzVector2(params[0], params[1]);
		p1 = CzVector2(seg->edgels[idx-point_used].p.x, seg->edgels[idx-point_used].p.y);
		p2 = CzVector2(seg->edgels[idx].p.x, seg->edgels[idx].p.y);
		dirP = p2-p1;
		dirP.Normalise();
		
		if( (dir.x < 0 && dirP.x > 0) || (dir.x > 0 && dirP.x < 0)) dir.x = -dir.x;
		if( (dir.y < 0 && dirP.y > 0) || (dir.y > 0 && dirP.y < 0)) dir.y = -dir.y;

		voteImg->InitLine(cvRound(seg->edgels[idx-points_used+1].p.x), cvRound(seg->edgels[idx-points_used+1].p.y), cvRound(seg->edgels[idx-points_used+1].p.x + dir.x*len), cvRound(seg->edgels[idx-points_used+1].p.y + dir.y*len), sline + VOTE_TE);
	}

	/*IplImage *drwImg = cvCreateImage(cvSize(1600,1024), IPL_DEPTH_8U, 3);
	cvResize(img, drwImg);
	cvNamedWindow("prova");
	cvShowImage("prova", drwImg);
	cvWaitKey(0);*/

   // grow search lines numGrow px
   z=0;
	while (z<numGrow) {
      for (unsigned i=0; i<segments.Size(); i++) {
         seg = segments[i];
         if (seg->edgels.Size()<min_points || seg->closed || seg->closable) {
            continue;
         }
         if (ContourClosed(seg, true)) {
		      seg->closable = true;
				   continue;
		   }

         sline = seg->id*8;
         if ((seg->start == NULL) && voteImg->ExtendLine(sline + VOTE_TS, iscts)) {
			   CzVoteImage::CzElem e;
			   for (unsigned ii=0; ii < iscts.Size(); ii++) {						
				   e = iscts[ii];
				   if (e.id%8 == 2 || e.id%8 == 3) {
					   //cout << "TS of" << seg->id << " intersects" << (int)e.id/8 << " Grow:" << z << endl;								
   					seg->start = segments[(int)e.id/8];
	   				if(e.id%8 == 2) {
		   				seg->s = false;
			   			segments[(int)e.id/8]->start = seg;
				   		segments[(int)e.id/8]->s = false;
					   } else {
						   seg->s = true;
   						segments[(int)e.id/8]->end = seg;
	   					segments[(int)e.id/8]->e = true;
		   			}				
   				}
	   		}
         }

         if ((seg->end == NULL) && voteImg->ExtendLine(sline + VOTE_TE, iscts)) {
	   		CzVoteImage::CzElem e;
		   	for (unsigned ii=0; ii < iscts.Size(); ii++) {
			   	e = iscts[ii];
				   if (e.id%8 == 2 || e.id%8 == 3) {
					   seg->end = segments[(int)e.id/8];
   					if (e.id%8 == 2) { //TE intersects with TS
	   					seg->e = true;
		   				segments[(int)e.id/8]->start = seg;
			   			segments[(int)e.id/8]->s = true;
				   	} else { //TE intersects with TE
					   	//cout << "TE of" << seg->id << " intersects with TE of" << (int)e.id/8 << " Grow:" << z << endl;							
						   seg->e = false;
   						segments[(int)e.id/8]->end = seg;
	   					segments[(int)e.id/8]->e = false;
		   			}				
			   	}
			   }
         }
      }
      z++;
   }
   cvReleaseMat(&points);
}

////////////////////////////////////////////////////////////////////////////////
// Draw the vote image onto the image img (for debugging purposes)
void CzFormClosedSegmentGroups::DrawVoteImg(IplImage *img, bool use_colour)
{
  CzVoteImage *vi = voteImg;
  if(vi == 0 || img==0)
    return;
  if ((use_colour && img->nChannels!=3)||
      (!use_colour && img->nChannels!=1)||
      img->depth!=IPL_DEPTH_8U)
    return;
  if (img->width!=vi->width || img->height!=vi->height)
    return;

  cvZero(img);

  for(int x = 0; x < vi->width; x++)
    for(int y = 0; y < vi->height; y++)
    {
      CzVoteImage::CzElem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        unsigned vtype = el->id%8;
        switch(vtype)
        {
          case VOTE_TS:
          case VOTE_NLS:
          case VOTE_NRS:
            if (use_colour){
              ((uchar*)(img->imageData+img->widthStep*y))[x*3] = 255; //magenta
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+1] = 0;
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+2] = 255;
            }else{
              ((uchar*)(img->imageData+img->widthStep*y))[x] = 255;
            }
            break;
          case VOTE_TE:
          case VOTE_NLE:
          case VOTE_NRE:
            if (use_colour){
              ((uchar*)(img->imageData+img->widthStep*y))[x*3] = 0; //cyan 
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+1] = 255;
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+2] = 255;
            }else{
              ((uchar*)(img->imageData+img->widthStep*y))[x] = 255;
            }
            break;
          default:
            if (use_colour){
              ((uchar*)(img->imageData+img->widthStep*y))[x*3] = 255; //white
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+1] = 255;
              ((uchar*)(img->imageData+img->widthStep*y))[x*3+2] = 255;
            }else{
              ((uchar*)(img->imageData+img->widthStep*y))[x] = 255;
            }
        }
        el = el->next;
      }
    } 
}

NAMESPACE_CLASS_END()

