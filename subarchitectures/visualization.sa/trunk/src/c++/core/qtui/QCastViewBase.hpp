/*
 * @author:  Marko Mahnič
 * @created: 2010-05-11
 */
#ifndef QCASTVIEWBASE_MGC6RE0Z
#define QCASTVIEWBASE_MGC6RE0Z

#include "../Model.hpp"
#include <QWidget>

/*interface*/
class QCastViewBase: public cogx::display::CDisplayModelObserver
{
public:
   virtual void setView(cogx::display::CDisplayView* pDisplayView) = 0;
   virtual operator QWidget&() = 0;
};


#endif /* end of include guard: QCASTVIEWBASE_MGC6RE0Z */
