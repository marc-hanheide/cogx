#ifndef RCVISUALIZER_H
#define RCVISUALIZER_H

#include <QtGui/QDialog>
#include "ui_RCVisualizer.h"

class QGraphicsScene;

namespace conceptual
{
	class Tester;
}
class ConceptualWidget;

class RCVisualizer : public QDialog
{
    Q_OBJECT

public:
    RCVisualizer(ConceptualWidget *parent, conceptual::Tester *component);
    ~RCVisualizer();


private slots:
	void saveImageButtonClicked();
	void generate();
	void addGroundtruthButtonClicked();


private:
    QBrush getBrushForProbability(double prob);


private:
    Ui::RCVisualizerClass ui;
    ConceptualWidget *_parent;
    conceptual::Tester *_component;

    /** Map roomId -> roomCateogory index. */
    std::map<int, int> _groundTruth;

};

#endif // RCVISUALIZER_H
