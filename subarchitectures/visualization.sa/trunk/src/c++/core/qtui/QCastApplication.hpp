/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 */

#ifndef __QCAST_APPLICATION_HPP__
#define __QCAST_APPLICATION_HPP__

#include <QApplication>
#include "../CDisplayServer.hpp"

namespace cogx { namespace display {

class QCastApplication: public QApplication
{
   Q_OBJECT // http://qtnode.net/wiki/Qt4_with_cmake
   cogx::display::CDisplayServer *m_pOwner;
public:
   QCastApplication(int &argc, char **argv, cogx::display::CDisplayServer *pOwner);

private slots:
   void checkIsRunning();

};

}} // namespace
#endif
