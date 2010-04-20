/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#include "QCastApplication.hpp"
#include <QTimer>

namespace cogx { namespace display {

QCastApplication::QCastApplication(int &argc, char **argv, cogx::display::CDisplayServer *pOwner)
   : QApplication(argc, argv)
{
   m_pOwner = pOwner;
   QTimer *timer = new QTimer(this);
   connect(timer, SIGNAL(timeout()), this, SLOT(checkIsRunning()));
   timer->start(1000);
}

void QCastApplication::checkIsRunning()
{
   if (! m_pOwner->isRunning()) {
      quit();
   }
}

}} // namespace
