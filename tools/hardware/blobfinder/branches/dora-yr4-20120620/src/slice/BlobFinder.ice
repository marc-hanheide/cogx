/**
 * Module to define data structures for a blob finder (like a CMU cam).
 * 
 */

#ifndef BLOBFINDER_ICE
#define BLOBFINDER_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module blobfinder {

  /** 
   * RGB color
   * 
   * values from 0 to 255.
   */
  struct ColorRGB {
    int r;
    int g;
    int b;
  };

  /**
   * Info on blob from finder.
   *
   * http://playerstage.sourceforge.net/doc/Player-2.1.0/player/structplayer__blobfinder__blob.html   
   */
  struct BlobInfo {
    ///Blob id assigned by the finder
    int id;
    ///A colour for the blob
    ColorRGB colour;
    ///Blob area in pixels
    int area;
    ///The blob bounding box
    cogx::Math::Rect2 boundingBox;
    ///The range to the blob in some unit. Player docs says metres, I disagree. Looks like millimetres to me.
    float range;    
  };

  
  sequence<BlobInfo> BlobInfoSequence;  
    
  
  /** This is the interface for getting blob info  **/

  interface BlobFinderInterface {

    /**
     * Get the number of blobs visible.
     */  
    ["cpp:const"] idempotent int getBlobCount();

    /**
     * Get all visible blobs
     */
    ["cpp:const"] idempotent BlobInfoSequence getBlobs();

  };


};

#endif // BLOBFINDER_ICE
