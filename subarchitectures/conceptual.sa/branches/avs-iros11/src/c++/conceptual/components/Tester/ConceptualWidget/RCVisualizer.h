#ifndef RCVISUALIZER_H
#define RCVISUALIZER_H

#include <QtGui/QDialog>
#include "ui_RCVisualizer.h"

class QGraphicsScene;

namespace conceptual
{
	class Tester;
}

class RCVisualizer : public QDialog
{
    Q_OBJECT

public:
    RCVisualizer(QWidget *parent, conceptual::Tester *component);
    ~RCVisualizer();

public slots:
	void saveImageButtonClicked();

private:
    void generate();

private:
    Ui::RCVisualizerClass ui;
    conceptual::Tester *_component;
};

#endif // RCVISUALIZER_H
