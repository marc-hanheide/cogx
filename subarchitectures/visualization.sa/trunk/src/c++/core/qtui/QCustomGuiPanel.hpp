/*
 * Author: Marko Mahnič
 * Created: 2010-04-21
 */

#ifndef QCUSTOMGUIPANEL_V9N1BGY9
#define QCUSTOMGUIPANEL_V9N1BGY9

#include <QFrame>
#include "../Model.hpp"

class QCustomGuiPanel: public QFrame, public cogx::display::CDisplayModelObserver
{
   Q_OBJECT
private:
   cogx::display::CDisplayView* m_pView;

private:
   void removeUi();

public:
   QCustomGuiPanel( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
   ~QCustomGuiPanel();
   void updateUi(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pView);
   bool hasView(cogx::display::CDisplayView *pView) {
      return pView == m_pView;
   }

public:
   // CDisplayModelObserver
   void onUiDataChanged(cogx::display::CDisplayModel *pModel, cogx::display::CDisplayView *pSourceView,
         cogx::display::CGuiElement *pElement, const std::string& newValue); /*override*/
};

#endif /* end of include guard: QCUSTOMGUIPANEL_V9N1BGY9 */
