/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * Task3ResultsPresenter.java
 *
 * Created on 2010-04-29, 20:15:33
 */
package binder.components;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import javax.swing.UIManager;
import violetsound.AePlayWave;

/**
 *
 *  * in which place are the record? indicating the place-id  for each object (referred to by its name)
 * which record belongs to whom? Represented by the record name and the person name
 * in which place is each person? indicating the place-id for each person (referred to by her/his name)
 * in which room is each record? Represented by the record name and the room name
 * in which room is each person? Represented by the person name and the room name
 *
 * @author Benoit Larochelle
 *
 * Behaviour undefined for more than 4 items
 */
public class Task3ResultsPresenter extends javax.swing.JFrame
{
    private static Task3ResultsPresenter instance = new Task3ResultsPresenter();

    private static Map<String, Person> persons = new HashMap<String, Person>(4);
    private static Map<String, Record> records = new HashMap<String, Record>(4);

    private static String[] personsSounds = {"turret-i_see_you", "turret-who_are_you", "turret-hello_friend", "turret-target_acquired"};
    private static String[] recordsSounds = {"curiosity-what_is_thaat", "curiosity-what_is_that", "curiosity-whats_thaat", "curiosity-oh_whats_that"};

    public static Task3ResultsPresenter getInstance()
    {
        return instance;
    }

    /** Creates new form Task3ResultsPresenter */
    private Task3ResultsPresenter()
    {
        initComponents();


        try
        {
            UIManager.setLookAndFeel("com.sun.java.swing.plaf.windows.WindowsLookAndFeel");
        } catch (Exception e)
        {
        }

    }

    public static boolean setPerson(String name, String place, String room, String record)
    {
        boolean alreadyExist = persons.containsKey(name);

        if(!alreadyExist)
        {
            //new AePlayWave("i:/wavs/" + personsSounds[persons.size()] + ".wav").start();
            new AePlayWave("wavs/" + personsSounds[persons.size()] + ".wav").start();
        }

        Task3ResultsPresenter.persons.put(name, new Person(name, place, room, record));
        updateUI();
        return alreadyExist;
    }

    public static boolean setRecord(String name, String place, String room, String owner)
    {
        boolean alreadyExist = persons.containsKey(name);

        if(!alreadyExist)
        {
            //new AePlayWave("i:/wavs/" + recordsSounds[persons.size()] + ".wav").start();
            new AePlayWave("wavs/" + recordsSounds[persons.size()] + ".wav").start();
        }

        Task3ResultsPresenter.records.put(name, new Record(name, place, room, owner));
        updateUI();
        return alreadyExist;
    }

    private static void updateUI()
    {
        Iterator<Person> itPersons = persons.values().iterator();

        if(itPersons.hasNext())
        {
            Person p = itPersons.next();
            instance.txtPerson1_Name.setText(p.name);
            instance.txtPerson1_Place.setText(p.place);
            instance.txtPerson1_Room.setText(p.room);
            instance.txtPerson1_Record.setText(p.record);
        }

        if(itPersons.hasNext())
        {
            Person p = itPersons.next();
            instance.txtPerson2_Name.setText(p.name);
            instance.txtPerson2_Place.setText(p.place);
            instance.txtPerson2_Room.setText(p.room);
            instance.txtPerson2_Record.setText(p.record);
        }

        if(itPersons.hasNext())
        {
            Person p = itPersons.next();
            instance.txtPerson3_Name.setText(p.name);
            instance.txtPerson3_Place.setText(p.place);
            instance.txtPerson3_Room.setText(p.room);
            instance.txtPerson3_Record.setText(p.record);
        }

        if(itPersons.hasNext())
        {
            Person p = itPersons.next();
            instance.txtPerson4_Name.setText(p.name);
            instance.txtPerson4_Place.setText(p.place);
            instance.txtPerson4_Room.setText(p.room);
            instance.txtPerson4_Record.setText(p.record);
        }

        Iterator<Record> itRecords = records.values().iterator();

        if(itRecords.hasNext())
        {
            Record r = itRecords.next();
            instance.txtRecord1_Name.setText(r.name);
            instance.txtRecord1_Place.setText(r.place);
            instance.txtRecord1_Room.setText(r.room);
            instance.txtRecord1_Owner.setText(r.owner);
        }

        if(itRecords.hasNext())
        {
            Record r = itRecords.next();
            instance.txtRecord2_Name.setText(r.name);
            instance.txtRecord2_Place.setText(r.place);
            instance.txtRecord2_Room.setText(r.room);
            instance.txtRecord2_Owner.setText(r.owner);
        }

        if(itRecords.hasNext())
        {
            Record r = itRecords.next();
            instance.txtRecord3_Name.setText(r.name);
            instance.txtRecord3_Place.setText(r.place);
            instance.txtRecord3_Room.setText(r.room);
            instance.txtRecord3_Owner.setText(r.owner);
        }
        
        if(itRecords.hasNext())
        {
            Record r = itRecords.next();
            instance.txtRecord4_Name.setText(r.name);
            instance.txtRecord4_Place.setText(r.place);
            instance.txtRecord4_Room.setText(r.room);
            instance.txtRecord4_Owner.setText(r.owner);
        }
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jLabel1 = new javax.swing.JLabel();
        jPanel1 = new javax.swing.JPanel();
        jLabel2 = new javax.swing.JLabel();
        jLabel3 = new javax.swing.JLabel();
        jLabel4 = new javax.swing.JLabel();
        jLabel5 = new javax.swing.JLabel();
        txtPerson1_Name = new javax.swing.JTextField();
        txtPerson1_Place = new javax.swing.JTextField();
        txtPerson1_Room = new javax.swing.JTextField();
        txtPerson1_Record = new javax.swing.JTextField();
        txtPerson2_Name = new javax.swing.JTextField();
        txtPerson2_Place = new javax.swing.JTextField();
        txtPerson2_Room = new javax.swing.JTextField();
        txtPerson2_Record = new javax.swing.JTextField();
        txtPerson3_Name = new javax.swing.JTextField();
        txtPerson3_Place = new javax.swing.JTextField();
        txtPerson3_Room = new javax.swing.JTextField();
        txtPerson3_Record = new javax.swing.JTextField();
        txtPerson4_Name = new javax.swing.JTextField();
        txtPerson4_Place = new javax.swing.JTextField();
        txtPerson4_Room = new javax.swing.JTextField();
        txtPerson4_Record = new javax.swing.JTextField();
        jPanel2 = new javax.swing.JPanel();
        jLabel6 = new javax.swing.JLabel();
        jLabel7 = new javax.swing.JLabel();
        jLabel8 = new javax.swing.JLabel();
        jLabel9 = new javax.swing.JLabel();
        txtRecord1_Name = new javax.swing.JTextField();
        txtRecord1_Place = new javax.swing.JTextField();
        txtRecord1_Room = new javax.swing.JTextField();
        txtRecord1_Owner = new javax.swing.JTextField();
        txtRecord2_Name = new javax.swing.JTextField();
        txtRecord2_Place = new javax.swing.JTextField();
        txtRecord2_Room = new javax.swing.JTextField();
        txtRecord2_Owner = new javax.swing.JTextField();
        txtRecord3_Name = new javax.swing.JTextField();
        txtRecord3_Place = new javax.swing.JTextField();
        txtRecord3_Room = new javax.swing.JTextField();
        txtRecord3_Owner = new javax.swing.JTextField();
        txtRecord4_Room = new javax.swing.JTextField();
        txtRecord4_Name = new javax.swing.JTextField();
        txtRecord4_Place = new javax.swing.JTextField();
        txtRecord4_Owner = new javax.swing.JTextField();
        jLabel10 = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setTitle("Task 3 Results");
        setResizable(false);

        jLabel1.setFont(new java.awt.Font("Tahoma", 1, 18)); // NOI18N
        jLabel1.setForeground(new java.awt.Color(153, 0, 153));
        jLabel1.setText("Task 3 Results");

        jPanel1.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Persons", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 0, 11), new java.awt.Color(153, 0, 153))); // NOI18N

        jLabel2.setText("Name");

        jLabel3.setText("Place");

        jLabel4.setText("Room");

        jLabel5.setText("Record");

        txtPerson1_Name.setEditable(false);

        txtPerson1_Place.setEditable(false);

        txtPerson1_Room.setEditable(false);

        txtPerson1_Record.setEditable(false);

        txtPerson2_Name.setEditable(false);

        txtPerson2_Place.setEditable(false);

        txtPerson2_Room.setEditable(false);

        txtPerson2_Record.setEditable(false);

        txtPerson3_Name.setEditable(false);

        txtPerson3_Place.setEditable(false);

        txtPerson3_Room.setEditable(false);

        txtPerson3_Record.setEditable(false);

        txtPerson4_Name.setEditable(false);

        txtPerson4_Place.setEditable(false);

        txtPerson4_Room.setEditable(false);

        txtPerson4_Record.setEditable(false);

        javax.swing.GroupLayout jPanel1Layout = new javax.swing.GroupLayout(jPanel1);
        jPanel1.setLayout(jPanel1Layout);
        jPanel1Layout.setHorizontalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(txtPerson1_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel2)
                    .addComponent(txtPerson2_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson3_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson4_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addGap(10, 10, 10)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(txtPerson1_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel3)
                    .addComponent(txtPerson2_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson3_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson4_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(txtPerson1_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel4)
                    .addComponent(txtPerson2_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson3_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson4_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel5)
                    .addComponent(txtPerson1_Record, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson2_Record, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson3_Record, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(txtPerson4_Record, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel1Layout.setVerticalGroup(
            jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel1Layout.createSequentialGroup()
                .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addGroup(jPanel1Layout.createSequentialGroup()
                        .addComponent(jLabel5)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson1_Record, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson2_Record, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson3_Record, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson4_Record, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(jPanel1Layout.createSequentialGroup()
                        .addComponent(jLabel4)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson1_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson2_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson3_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(txtPerson4_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                        .addGroup(jPanel1Layout.createSequentialGroup()
                            .addComponent(jLabel3)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson1_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson2_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson3_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson4_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGroup(jPanel1Layout.createSequentialGroup()
                            .addComponent(jLabel2)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson1_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson2_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson3_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                            .addComponent(txtPerson4_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))))
                .addContainerGap(10, Short.MAX_VALUE))
        );

        jPanel2.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Records", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 0, 11), new java.awt.Color(153, 0, 153))); // NOI18N

        jLabel6.setText("Name");

        jLabel7.setText("Place");

        jLabel8.setText("Room");

        jLabel9.setText("Owner");

        txtRecord1_Name.setEditable(false);

        txtRecord1_Place.setEditable(false);

        txtRecord1_Room.setEditable(false);

        txtRecord1_Owner.setEditable(false);

        txtRecord2_Name.setEditable(false);

        txtRecord2_Place.setEditable(false);

        txtRecord2_Room.setEditable(false);

        txtRecord2_Owner.setEditable(false);

        txtRecord3_Name.setEditable(false);

        txtRecord3_Place.setEditable(false);

        txtRecord3_Room.setEditable(false);

        txtRecord3_Owner.setEditable(false);

        txtRecord4_Room.setEditable(false);

        txtRecord4_Name.setEditable(false);

        txtRecord4_Place.setEditable(false);

        txtRecord4_Owner.setEditable(false);

        javax.swing.GroupLayout jPanel2Layout = new javax.swing.GroupLayout(jPanel2);
        jPanel2.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(txtRecord1_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel6)
                            .addComponent(txtRecord2_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord3_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(txtRecord1_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel7)
                            .addComponent(txtRecord2_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord3_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addComponent(txtRecord4_Name, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addComponent(txtRecord4_Place, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(txtRecord1_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(jLabel8)
                            .addComponent(txtRecord2_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord3_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(jLabel9)
                            .addComponent(txtRecord1_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord2_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord3_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addComponent(txtRecord4_Room, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addComponent(txtRecord4_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, 123, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(jPanel2Layout.createSequentialGroup()
                .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(jPanel2Layout.createSequentialGroup()
                                .addComponent(jLabel9)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord1_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord2_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord3_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(jPanel2Layout.createSequentialGroup()
                                .addComponent(jLabel8)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord1_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord2_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord3_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(txtRecord4_Owner, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord4_Room, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(jPanel2Layout.createSequentialGroup()
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(jPanel2Layout.createSequentialGroup()
                                .addComponent(jLabel6)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord1_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord2_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord3_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(jPanel2Layout.createSequentialGroup()
                                .addComponent(jLabel7)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord1_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord2_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(txtRecord3_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(txtRecord4_Name, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(txtRecord4_Place, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );

        jLabel10.setFont(new java.awt.Font("Tahoma", 1, 48)); // NOI18N
        jLabel10.setForeground(new java.awt.Color(153, 0, 153));
        jLabel10.setText("•");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(jPanel2, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addGroup(javax.swing.GroupLayout.Alignment.LEADING, layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
                        .addGroup(javax.swing.GroupLayout.Alignment.LEADING, layout.createSequentialGroup()
                            .addComponent(jLabel10)
                            .addGap(182, 182, 182)
                            .addComponent(jLabel1))
                        .addComponent(jPanel1, javax.swing.GroupLayout.Alignment.LEADING, 0, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
                .addContainerGap(14, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(jLabel10, javax.swing.GroupLayout.PREFERRED_SIZE, 30, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(jLabel1))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(jPanel1, javax.swing.GroupLayout.PREFERRED_SIZE, 155, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(18, 18, 18)
                .addComponent(jPanel2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(27, Short.MAX_VALUE))
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    /**
     * @param args the command line arguments
     */
    public static void main(String args[])
    {
        java.awt.EventQueue.invokeLater(new Runnable()
        {

            public void run()
            {
                new Task3ResultsPresenter().setVisible(true);
            }
        });
    }

    private static class Record
    {

        public String name;
        public String place;
        public String room;
        public String owner;

        public Record(String name, String place, String room, String owner)
        {
            this.name = name;
            this.place = place;
            this.room = room;
            this.owner = owner;
        }

    }

    private static class Person
    {

        public String name;
        public String place;
        public String room;
        public String record;

        public Person(String name, String place, String room, String record)
        {
            this.name = name;
            this.place = place;
            this.room = room;
            this.record = record;
        }

    }


    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JTextField txtPerson1_Name;
    private javax.swing.JTextField txtPerson1_Place;
    private javax.swing.JTextField txtPerson1_Record;
    private javax.swing.JTextField txtPerson1_Room;
    private javax.swing.JTextField txtPerson2_Name;
    private javax.swing.JTextField txtPerson2_Place;
    private javax.swing.JTextField txtPerson2_Record;
    private javax.swing.JTextField txtPerson2_Room;
    private javax.swing.JTextField txtPerson3_Name;
    private javax.swing.JTextField txtPerson3_Place;
    private javax.swing.JTextField txtPerson3_Record;
    private javax.swing.JTextField txtPerson3_Room;
    private javax.swing.JTextField txtPerson4_Name;
    private javax.swing.JTextField txtPerson4_Place;
    private javax.swing.JTextField txtPerson4_Record;
    private javax.swing.JTextField txtPerson4_Room;
    private javax.swing.JTextField txtRecord1_Name;
    private javax.swing.JTextField txtRecord1_Owner;
    private javax.swing.JTextField txtRecord1_Place;
    private javax.swing.JTextField txtRecord1_Room;
    private javax.swing.JTextField txtRecord2_Name;
    private javax.swing.JTextField txtRecord2_Owner;
    private javax.swing.JTextField txtRecord2_Place;
    private javax.swing.JTextField txtRecord2_Room;
    private javax.swing.JTextField txtRecord3_Name;
    private javax.swing.JTextField txtRecord3_Owner;
    private javax.swing.JTextField txtRecord3_Place;
    private javax.swing.JTextField txtRecord3_Room;
    private javax.swing.JTextField txtRecord4_Name;
    private javax.swing.JTextField txtRecord4_Owner;
    private javax.swing.JTextField txtRecord4_Place;
    private javax.swing.JTextField txtRecord4_Room;
    // End of variables declaration//GEN-END:variables
}
