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
struct view_settings
{
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
    void load(const std::string &filename);
};

class ControlWindow : public Gtk::Window
{
public:
    ControlWindow();
    virtual ~ControlWindow();
protected:
    class ModelColumns : public Gtk::TreeModel::ColumnRecord
    {
    public:

    ModelColumns()
    { add(m_col_id); add(m_col_name); }
    Gtk::TreeModelColumn<unsigned int> m_col_id;
    Gtk::TreeModelColumn<Glib::ustring> m_col_name;
    };
  
    ModelColumns m_Columns;
    Gtk::VBox m_box1;
    //The Tree model:
    Glib::RefPtr<Gtk::ListStore> m_refTreeModel;
    Glib::RefPtr<Gtk::TreeSelection> m_refTreeSelection;
    //Member widgets:
    Gtk::TreeView m_TreeView;
private:
    Gtk::CheckButton follow_robot;
    Glib::Thread * thread;

    std::map<std::string,Gtk::CheckButton* > toggles;
    virtual void on_selection_changed();
    void on_button_clicked();
    void run();
    pb_settings ps;
    peekabot::PeekabotClient m_PeekabotClient;
    void connectPeekabot();
};

#endif // PBDisplayControl_hpp
