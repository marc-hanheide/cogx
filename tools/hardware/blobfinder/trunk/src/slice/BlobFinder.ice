/**
 * Module to define data structures for a blob finder (like a CMU cam).
 * 
 */

#ifndef BLOBFINDER_ICE
#define BLOBFINDER_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module blobfinder {

  /** RGB color
   * NOTE: bytes in ICE are -128..127! So you will need to cast to an unsigned
   * char in your code.
   *
   * TODO: this is copied from VisionData.ice and should probably be moved to somewhere more general
   */
  struct ColorRGB {
    byte r;
    byte g;
    byte b;
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
