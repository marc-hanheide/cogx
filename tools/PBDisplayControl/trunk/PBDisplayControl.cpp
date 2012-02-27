#include "PBDisplayControl.hpp"
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <iostream>

using namespace std;
using namespace Gtk;



void pb_settings::load(const std::string &filename)
{

    // Create empty property tree object
    using boost::property_tree::ptree;
    ptree pt;

    // Load XML file and put its contents in property tree. 
    // No namespace qualification is needed, because of Koenig 
    // lookup on the second argument. If reading fails, exception
    // is thrown.
    read_xml(filename, pt);

    // Iterate over debug.modules section and store all found 
    // modules in m_modules set. get_child() function returns a 
    // reference to child at specified path; if there is no such 
    // child, it throws. Property tree iterator can be used in 
    // the same way as standard container iterator. Category 
    // is bidirectional_iterator.
    BOOST_FOREACH(ptree::value_type const& v, pt.get_child("pb_settings.views")){
        view_settings vs;
        vs.name = v.second.get<std::string>("name");
        vs.camera.x = v.second.get<double>("camera.x",0);
        vs.camera.y = v.second.get<double>("camera.y",0);
        vs.camera.z = v.second.get<double>("camera.z",0);

        vs.camera.yaw = v.second.get<double>("camera.yaw",0);
        vs.camera.pitch = v.second.get<double>("camera.pitch",0);
        vs.camera.roll = v.second.get<double>("camera.roll",0);

        vs.camera.glob = v.second.get<bool>("camera.glob",true);
        vs.camera.orthographic = v.second.get<bool>("camera.orthographic",false);

        BOOST_FOREACH(ptree::value_type const&v1, v.second.get_child("objects"))
            vs.objects[v1.second.data()]=v1.second.get<bool>("<xmlattr>.hidden", false);
        views.push_back(vs);
    }

    BOOST_FOREACH(ptree::value_type const&v1, pt.get_child("pb_settings.toggles")){
        toggles.push_back(v1.second.data());
    }
    m_PbPort = pt.get<int>("pb_settings.pbport",5050);
    m_PbHost = pt.get<std::string>("pb_settings.pbhost","localhost");
}


ControlWindow::ControlWindow() : follow_robot("follow the robot") {
  set_title("PBDisplayControl");
  set_default_size(200, 200);
  // Sets the border width of the window.
  set_border_width(10); 
  add(m_box1);
  m_refTreeModel = Gtk::ListStore::create(m_Columns);
  m_TreeView.set_model(m_refTreeModel);

    ps.load("../../instantiations/PBDisplayControl/pb_settings.xml");
    connectPeekabot();
    for (int i=0; i<ps.views.size(); i++) {
      Gtk::TreeModel::Row row = *(m_refTreeModel->append());
      row[m_Columns.m_col_id] = i;
      row[m_Columns.m_col_name] = ps.views[i].name;
    }

    m_box1.pack_start(follow_robot);

follow_robot.show();
follow_robot.set_active(true);

    for (int i=0; i<ps.toggles.size(); i++) {
        Gtk::CheckButton* toggle = new Gtk::CheckButton(ps.toggles[i]);
        toggle->signal_clicked().connect(sigc::mem_fun(*this,
              &ControlWindow::on_button_clicked) );
        toggles[ps.toggles[i]]=toggle;
        m_box1.pack_start(*toggle);
        toggle->show();
    }
  //Fill the TreeView's model

  //Add the TreeView's view columns:
//  append_column("ID", m_Columns.m_col_id);
  m_TreeView.append_column("Choose view", m_Columns.m_col_name);

  m_box1.pack_start(m_TreeView);
    m_TreeView.show();
   m_refTreeSelection = m_TreeView.get_selection();
   m_refTreeSelection->signal_changed().connect( sigc::mem_fun(*this, &ControlWindow::on_selection_changed) );
  // The final step is to display this newly created widget...
  thread = Glib::Thread::create(sigc::mem_fun(*this, &ControlWindow::run), true);
  
  //Glib::signal_timeout().connect( sigc::mem_fun(*this, &ControlWindow::on_timer),
  //        500 );

  set_keep_above();
  show();
  show_all_children();  
  ApplyView(0);    
}
ControlWindow::~ControlWindow(){}

void ControlWindow::run()
{
    while(true){
        double x=0;
        double y=0;
        double z=0;
        if (follow_robot.get_active()){
            peekabot::CameraProxy pb_camera;
            peekabot::Status s;
            peekabot::ObjectProxy pb_object;
            s = pb_object.assign(m_PeekabotClient,"SLAM_Pose").status();
            if( s.succeeded() ) {
                peekabot::Result<peekabot::Vector3> r = pb_object.get_position();
                if( r.succeeded() ){
                  peekabot::Vector3 v3 = r.get_result();
                  x = v3(0);
                  y = v3(1);
                  z = v3(2);
                }
            }
            s = pb_camera.assign(m_PeekabotClient,"default_camera").status();
            if( s.succeeded() ) {
                pb_camera.set_position(x,y,z);
            }
        }
    }
}

void ControlWindow::connectPeekabot()
{

  try {
    printf("Trying to connect to Peekabot on host %s and port %d\n",
        ps.m_PbHost.c_str(), ps.m_PbPort);
    
    m_PeekabotClient.connect(ps.m_PbHost, ps.m_PbPort);

    printf("Connection to Peekabot established\n");
    
  } catch(std::exception &e) {
    printf("Caught exception when connecting to peekabot (%s)\n",
        e.what());
    return;
  }
}

void ControlWindow::on_selection_changed(){
    TreeModel::iterator iter = m_refTreeSelection->get_selected();
    if(iter) //If anything is selected
    {
        TreeModel::Row row = *iter;

        int vid =row[m_Columns.m_col_id];
        ApplyView(vid);
    }
}

void ControlWindow::ApplyView(int vid){
        peekabot::CameraProxy pb_camera;
        peekabot::Status s;
        s = pb_camera.assign(m_PeekabotClient,"default_camera").status();
        if( s.succeeded() ) {
            if (!follow_robot.get_active()){            
                pb_camera.set_position(ps.views[vid].camera.x,ps.views[vid].camera.y,ps.views[vid].camera.z);
            }
            pb_camera.set_rotation(ps.views[vid].camera.yaw,ps.views[vid].camera.pitch,ps.views[vid].camera.roll);
            pb_camera.set_orthographic(ps.views[vid].camera.orthographic);
        }

        std::map<std::string, bool>::const_iterator iter1;
        for (iter1=ps.views[vid].objects.begin(); iter1 != ps.views[vid].objects.end(); ++iter1) {
            peekabot::ObjectProxy pb_object;
            s = pb_object.assign(m_PeekabotClient,iter1->first).status();
            if( s.succeeded() ) {
                if (iter1->second){
                    pb_object.hide();
                    if (toggles.find(iter1->first) != toggles.end())
                        toggles[iter1->first]->set_active(false);
                }
                else {
                    pb_object.show();
                    if (toggles.find(iter1->first) != toggles.end())
                        toggles[iter1->first]->set_active(true);
                }
            }
        }
}

void ControlWindow::on_button_clicked(){
     peekabot::Status s;
    std::map<std::string, Gtk::CheckButton*>::const_iterator iter1;
    for (iter1=toggles.begin(); iter1 !=toggles.end(); ++iter1) {
        peekabot::ObjectProxy pb_object;
        s = pb_object.assign(m_PeekabotClient,iter1->first).status();
        if( s.succeeded() ) {
            if (iter1->second->get_active())pb_object.show();
            else pb_object.hide();
        }
    }
}

int main(int argc, char *argv[]){
    Gtk::Main kit(argc, argv);
    ControlWindow cw;
    Gtk::Main::run(cw);
}
