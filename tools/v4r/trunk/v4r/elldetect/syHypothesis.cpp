//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syHypothesis.hpp"

NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
CzHypothesis::CzHypothesis(double x_in, double y_in, double a_in, double b_in, double phi_in)
{
   Set(x_in, y_in, a_in, b_in, phi_in);
}

////////////////////////////////////////////////////////////////////////////////
// Get the count of edgels
const int CzHypothesis::GetEdgelsCount()
{
   return m_pEdgels.Size();
}

////////////////////////////////////////////////////////////////////////////////
// Get edgel by index
const CzEdgel* CzHypothesis::GetEdgel( int iIndex )
{
   ASSERT( iIndex >= 0 );
   ASSERT( iIndex < (int) m_pEdgels.Size() );
   
   return &m_pEdgels[iIndex];
}


NAMESPACE_CLASS_END()
