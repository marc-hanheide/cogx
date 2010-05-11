/*
 * @author:  Marko Mahnič
 * @created: 2010-05-11
 */

#ifndef QVIEWCONTAINER_NXZGNUIJ
#define QVIEWCONTAINER_NXZGNUIJ

#include "QCastViewBase.hpp"
#include <QWidget>
#include <QFrame>

class QViewContainer: public QFrame
{
   Q_OBJECT
private:
   // TODO: array of displays when grid of views is supported
   QCastViewBase *m_pDisplay; // Currently active QCastViewXX

public:
   QViewContainer( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QViewContainer ();

   // TODO: multiple views in a grid? -> setView(index, pView)
   void setView(cogx::display::CDisplayView* pView);

private:
   void removeUi();
};

#endif /* end of include guard: QVIEWCONTAINER_NXZGNUIJ */
