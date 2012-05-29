#ifndef PBDisplayControl_hpp
#define PBDisplayControl_hpp

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <sigc++/sigc++.h>

#include <gtkmm.h>
#include <gtkmm/button.h>
#include <gtkmm/window.h>

#include <peekabot.hh>
#include <peekabot/Types.hh>
#include <map>
#include <string>

struct cam_settings {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  bool glob;
  bool orthographic;
};
struct view_settings {
  std::string name;
  cam_settings camera;
  std::map<std::string, bool> objects;
};

class pb_settings {
public:
  std::string m_PbHost;
  int m_PbPort;
  std::vector<view_settings> views;
  std::vector<std::string> toggles;
  bool follow_robot;
  void load(const std::string &filename);
};

class PeekabotController {
public:
  PeekabotController();

  peekabot::PeekabotClient m_PeekabotClient;
  pb_settings ps;
  void connectPeekabot();
  void ApplyView(int vid);
  void show(const std::string &label);
  void hide(const std::string &label);
  void follow();
};

class ControlWindow: public Gtk::Window {
public:
  ControlWindow(PeekabotController &_ctrl);
  virtual ~ControlWindow();
protected:
  class ModelColumns: public Gtk::TreeModel::ColumnRecord {
  public:

    ModelColumns() {
      add(m_col_id);
      add(m_col_name);
    }
    Gtk::TreeModelColumn<unsigned int> m_col_id;
    Gtk::TreeModelColumn<Glib::ustring> m_col_name;
  };

  void setActivationFromView(int vid);

  ModelColumns m_Columns;
  Gtk::VBox m_box1;
  //The Tree model:
  Glib::RefPtr<Gtk::ListStore> m_refTreeModel;
  Glib::RefPtr<Gtk::TreeSelection> m_refTreeSelection;
  //Member widgets:
  Gtk::TreeView m_TreeView;
private:
  PeekabotController &m_controller;

  Gtk::CheckButton follow_robot_button;
  Glib::Thread * thread;

  std::map<std::string, Gtk::CheckButton*> toggles;
  virtual void on_selection_changed();
  void on_button_clicked();
  void run();
};

#endif // PBDisplayControl_hpp
