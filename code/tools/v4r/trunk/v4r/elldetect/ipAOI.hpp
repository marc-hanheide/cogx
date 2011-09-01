#ifndef IPAOI_HPP
#define IPAOI_HPP



NAMESPACE_CLASS_BEGIN( CzN )

//begin class///////////////////////////////////////////////////////////////////
//
//    class description:
//
//       Base class for AOIs. 
//
//    function description:
//
//
//
////////////////////////////////////////////////////////////////////////////////
class CzAOI
{
   public:
                            CzAOI();
      virtual              ~CzAOI() = 0;
/*                        
               VOID         SetAOIName( const CzString& rName );
               CzString     GetAOIName() const;

               VOID         SetUIName(  const CzString& rName );
               CzString     GetUIName() const;

               VOID         SetAOIInfo( CzAOIInfo* pInfo );
               CzAOIInfo*   GetAOIInfo() const;
            
      virtual  VOID         SetAOIRect(  const CzsRectangle& rRect ) = 0;
      virtual  CzsRectangle GetAOIRect()     const = 0;
      virtual  CzsRectangle GetAOIMaxRect()  const = 0;
      virtual  CzsPoint     AOIPos()         const = 0;
      virtual  CzsSize      AOISize()        const = 0;
      virtual  SHORT        AOIWidth()       const = 0;
      virtual  SHORT        AOIHeight()      const = 0;

      virtual  BaseDevType  GetBaseDevType() const = 0;
      virtual  DeviceType   GetDevType()     const = 0;

      virtual  VOID         Show(       CzOutputDevice* pOutDev ) = 0;
      virtual  VOID         Show( const CzsRectangle&   rDstRect,
                                        CzOutputDevice* pOutDev ) = 0;
                                             
      virtual  VOID         ShowStretched( const CzsRectangle&   rSrcRect, 
                                                 CzOutputDevice* pOutDev ) const = 0;

      virtual  VOID         ShowStretched( const CzsRectangle&   rSrcRect, 
                                           const CzsRectangle&   rDstRect, 
                                                 CzOutputDevice* pOutDev ) const = 0;

  
      virtual  VOID         Load( const CzString& rFileName ) = 0;
      virtual  VOID         Save( const CzString& rFileName ) = 0;


   protected:
      CzString   aAOIName;
      CzString   aUIName;
      CzAOIInfo* pAOIInfo;

      FLOAT      GetZoomFactor( ZoomFactor eZoomFact ) const;

   private:                                     // Copy construction and
                 CzAOI( const CzAOI& rCopy );   // assign are prohibited by
      CzAOI& operator=( const CzAOI& rCopy );   // default to avoid problems
*/
};                                              // at thoughtless usage...
//end class/////////////////////////////////////////////////////////////////////

NAMESPACE_CLASS_END()

#endif
