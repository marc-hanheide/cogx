/*
 * Author: Marko Mahnič
 * Created: 2010-03-10
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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
