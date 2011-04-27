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
