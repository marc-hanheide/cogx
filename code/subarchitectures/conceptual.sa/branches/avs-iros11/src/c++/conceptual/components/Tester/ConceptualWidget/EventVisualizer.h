#ifndef RCVISUALIZER_H
#define RCVISUALIZER_H

#include "ui_EventVisualizer.h"
#include "ConceptualEvent.h"

#include <QtGui/QDialog>
#include <set>

class QGraphicsScene;
class ConceptualWidget;

class EventVisualizer : public QDialog
{
    Q_OBJECT

public:
    EventVisualizer(QWidget *parent,
    		const std::vector<std::string> &roomCats, const std::vector<std::string> &shapes, const std::vector<std::string> &sizes,
			const std::vector<std::string> &appearances, const std::vector<std::string> &visualizedObjects);
    ~EventVisualizer();

public slots:

	void generate(const QList<conceptual::ConceptualEvent> &events);


private slots:

	void generate();
	void saveSvgButtonClicked();
	void savePngButtonClicked();
	void addGroundtruthButtonClicked();




private:
    Ui::RCVisualizerClass ui;

	const std::vector<std::string> *_roomCats;
	int _curPlaceId;
	int _curRoomId;
	std::set<int> _roomIds;
	QList<conceptual::ConceptualEvent> _lastEvents;

};

#endif // RCVISUALIZER_H
