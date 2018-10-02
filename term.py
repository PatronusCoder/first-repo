#!/usr/bin/env python


"""
##
Author      : Pravin Kumar Jaisawal
Institution : Siegen University, Germany
##
"""

## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf # required to perform transformation from one frame to other
from sensor_msgs.msg import * # required for subscribing to joy
from Tkinter import * #for making gui
import os # for working with directory
import numpy as np # to calculate some mathematics operation. 
from decimal import Decimal
# this is required for multitasking. Specially my buttons get frozen, but i have not used it yet. I dont know how to use them yet
import threading
import thread

#these are used as the counters
j1 = 0
j2 = 0
j3 = 0
j4 = 0
j5 = 0
#these represent the flag that are raised after there is falling egde or when buttons are released in case of continuous movement
f1 = False
f2 = False
f3 = False
f4 = False
f5 = False
# these flags are used for stepwise movement
flag1 = True
flag2 = True
flag3 = True
flag4 = True
flag5 = True
flag6 = True
flag7 = True
flag8 = True
flag9 = True
flag10 = True


#reading all the names of poses from posenames.txt. This file is in catkin_ws
def update_joints_from_txt():
    os.chdir("/home/pravin/catkin_ws")
    with open('posenames.txt','r') as f11:
        comments=f11.readlines()
        #print comments
        var =[]
        values =[]
        for i in comments:
            a = i.strip()
            if a != '': # this check is to avoid evaluating empty strings
                a1 = eval(a)
            for key in a1:
                var.append(key)
                values.append(a1[key])
    return var, values

# this is written outside because it is easy to change font for all the data, otherwise i need to write in each label.
labelfont = (14)

#rotation matrix for rotation in X-axis
def rotx(roll):
    Rx = np.array([[1, 0, 0, 0], [0, np.cos(np.deg2rad(roll)), -np.sin(np.deg2rad(roll)), 0],
                   [0, np.sin(np.deg2rad(roll)), np.cos(np.deg2rad(roll)), 0], [0, 0, 0, 1]])
    return Rx

#rotation matrix for rotation in Y-axis
def roty(pitch):
    Ry = np.array([[np.cos(np.deg2rad(pitch)), 0, np.sin(np.deg2rad(pitch)), 0], [0, 1, 0, 0],
                   [-np.sin(np.deg2rad(pitch)), 0, np.cos(np.deg2rad(pitch)), 0], [0, 0, 0, 1]])
    return Ry

#rotation matrix for rotation in Z-axis
def rotz(yaw):
    Rz = np.array([[np.cos(np.deg2rad(yaw)), -np.sin(np.deg2rad(yaw)), 0, 0],
                   [np.sin(np.deg2rad(yaw)), np.cos(np.deg2rad(yaw)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return Rz

#transformation matrix
def tran(position):
    T1 = np.array([[1, 0, 0, position[0]], [0, 1, 0, position[1]], [0, 0, 1, position[2]], [0, 0, 0, 1]])
    return T1

#function to perform matrix multiplication
def matprod(transforms):
    F = np.identity(4)
    for item in transforms[:]:
        F = np.matmul(item, F)
    return F

# This class is used to work with new popup window    
class popupwindow:
    def __init__(self, popup):
        top = self.top = Toplevel(popup,bg = 'lightblue')
        #label to print sth, see below the content
        self.l = Label(top, bg = 'lightblue',text="Please enter the name of the pose", font =labelfont)
        # now it should be placed on the gui
        self.l.grid(row=0, column=0, sticky=W)
        # to get a box in the screen
        self.e = Entry(top)
        #to show it in the screen
        self.e.grid(row=0, column=1, columnspan=2, sticky=W)
        # to get button for the screen, text= sth(see below) will be written on it 
        self.b1 = Button(top,bg = 'lightblue', text="OK", command=self.getname)
        # to show it in the screen
        self.b1.grid(row=1, column=0, columnspan=2, sticky=N)
        #similar to b1, see above
        self.b2 = Button(top,bg = 'lightblue', text="Cancel", command=top.destroy)
        self.b2.grid(row=1, column=1, columnspan=2, sticky=N)
    #method to get the name from the box of the gui and also write it into posenames.txt also to srdf file if uncommented the section below
    def getname(self):
        self.count = 0
        #get the value
        self.value = self.e.get()
        #label if nothing is written or already present. Overwriting is not allowed now. To change the name, it should be done manually both from posenames.txt and srdf file if this file is also used
        #self.l3 = Label(self.top, text="  Error : No name given    ",font=labelfont)
        
        #print(self.l2)
        # to check if length is greater than zero. It means sth is entered into the box.May be single space without anything also works.This error handling is not done now. But could be implemented if needed.
        if len(self.value) > 0:
            #to check whether the name was already in use, get all the names from the posenames.txt and compare with the entered value
            os.chdir("/home/pravin/catkin_ws")
            with open('posenames.txt', 'r') as f1:
                poses1 = f1.readlines()
		pose_names =[]
		for i in poses1:
        		a = i.strip()
                        if a != '':
                            a1 = eval(a)
        		for key in a1:
            			pose_names.append(key)
                flag = 1 # this is used as it serves as a flag if everything is ok meaning the entered value is new name
                for item in pose_names:
                    #print(item)
                    if ((self.value) == item):
                        #print("The name already exists. Please try another name")
                        self.count = +1
                        if self.count > 1:
                            self.l2.destroy()
                        self.l2 = Label(self.top,bg = 'lightblue', text="Sorry, this name is already in use. Please give another name",font=labelfont)
                        self.l2.grid(row=2, column=0, columnspan=2, sticky=N) # show the error msg. see label l2 above
                        """if self.l3 in globals():
                            self.l3.destroy()"""
                        flag = 0 #flag is zero indicating the name is already in use. so no need to save
                        break #once flag is zero, no need to check with other names so exit the loop
                    else:
                        flag = 1

                if (flag):
                    # changing directory because posenames.txt is here
                    os.chdir("/home/pravin/catkin_ws")
		    pose_dict = {}
		    pose_dict[self.value]=group_variable_values
                    # simply add the entered name into the file at last.
                    os.chdir("/home/pravin/catkin_ws")
                    with open('posenames.txt', 'a') as f:
                        f.write(str(pose_dict)+"\n") 

                    self.top.destroy() # close the popup
                    """
                    os.chdir("/home/pravin/catkin_ws/src/igus_moveit_config/config")
                    with open('igus.srdf','r') as f:
                        with open('temp.txt', 'w') as ou:
                            for line in f.readlines():
                                if line =="    <!--DISABLE COLLISIONS: By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. -->\n":"""
                                    #ou.write("""    <group_state name=\""""+self.value+ """\" group="igus_planning"> \n""")
                                    #ou.write("""\t<joint name="ee_joint" value=\" %.4f\" /> \n""" %group_variable_values[4])
                                    #ou.write("""\t<joint name="j1" value=\" %.4f\" /> \n""" %group_variable_values[0])
                                    #ou.write("""\t<joint name="j2" value=\" %.4f\" /> \n""" %group_variable_values[1])
                                    #ou.write("""\t<joint name="j3" value=\" %.4f\" /> \n""" %group_variable_values[2])
                                    #ou.write("""\t<joint name="j4" value=\" %.4f\" /> \n""" %group_variable_values[3])
                                    #ou.write("""    </group_state> \n""")
                                    #ou.write(line)
                                #else:
                                    #ou.write(line)
                    #with open('temp.txt','r') as f:
                        #content =f.readlines()


                    #with open('igus.srdf','w') as f2:
                        #f2.writelines(content)"""                    
 
                    #just checking whether we are in corrrect directory    
                    print(os.getcwd())
                    os.chdir("/home/pravin/catkin_ws")
                    print(os.getcwd())
                    

                        
        else:
            self.count = +1
            if self.count > 1:
                self.l2.destroy()
            self.l2 = Label(self.top,bg = 'lightblue', text="  Error : No name given    ",font=labelfont,padx=200)
            self.l2.grid(row=2, column=0, columnspan=2,sticky=W) #show error label l3. see above
            #count = 1


# this is the main gui class. The above class is called using this class. #Instructions used here are similar to above. So comments are not used.Only when it is different, comments are shown
class gui:

    def __init__(self, gui_name):
        self.master = gui_name
        # global variables for counting the number of click on checkbutton
        global c1var,num,root
        c1var = IntVar()
        c1var = 0
        num = IntVar()
        #Frames to separate three sections
        self.frame_main = Frame(gui_name, highlightbackground="gray", highlightthickness=3, width=root.winfo_screenwidth(),height= root.winfo_screenheight()-200)
        self.frame_main.place(x=0, y =200)
        self.frame_main.grid_propagate(False)
        self.frame_sub1 =Frame(self.frame_main,width=700,bg = 'lightblue',height= root.winfo_screenheight())   
        self.frame_sub1.grid(row=0, column=0,sticky=W)
        self.frame_sub1.grid_propagate(False) 
        self.frame_sub2 =Frame(self.frame_main,width=root.winfo_screenwidth()-700,bg = 'lightblue',height=root.winfo_screenheight())   
        self.frame_sub2.grid(row=0, column=1,sticky=W)
        self.frame_sub2.grid_propagate(False)     
        self.f1 = Frame(self.frame_sub1,bg = 'lightblue',highlightbackground="gray", highlightthickness=3,width=700,height=root.winfo_screenheight()-500)        
        self.f1.grid(row=0, column=0,sticky=W)
        self.f1.grid_propagate(False)
        self.f2 = Frame(self.frame_sub2,bg = 'lightblue', relief =SUNKEN,highlightbackground="gray", highlightthickness=3,width=root.winfo_screenwidth()-700, height=200)
        self.f2.grid(row=0, column=0,sticky=W)
        self.f2.grid_propagate(False)
        self.f3 = Frame(self.frame_sub1,bg = 'lightblue',highlightbackground="gray", highlightthickness=3,width=700,height=500)
        self.f3.grid(row=1, column=0,sticky=W)
        self.f3.grid_propagate(False)
        self.f4 = Frame(self.frame_sub2,bg = 'lightblue',highlightbackground="gray", highlightthickness=3,width=root.winfo_screenwidth()-700,height=root.winfo_screenheight()-400)
        self.f4.grid(row=1, column=0,sticky=W)
        self.f4.grid_propagate(False)
        #All that should be in frame 1
        self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]), anchor= 'w',font=labelfont)
        self.label1.grid(row=0,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)

        self.label2 = Label(self.f1, text="Please enter the desired angles for each joints in degree:",font=labelfont,bg = 'lightblue')
        self.label2.grid(row=3, column=0, columnspan=2, sticky=W)

        self.Joint1 = Label(self.f1, text="Joint1 :",font=labelfont,bg = 'lightblue')
        self.Joint1.grid(row=4, column=0, sticky=E)
        self.value1 = Entry(self.f1,font=labelfont)
        self.value1.grid(row=4, column=1, sticky=W)

        self.Joint2 = Label(self.f1, text="Joint2 :",font=labelfont,bg = 'lightblue')
        self.Joint2.grid(row=5, column=0, sticky=E)
        self.value2 = Entry(self.f1,font=labelfont)
        self.value2.grid(row=5, column=1, sticky=W)

        self.Joint3 = Label(self.f1, text="Joint3 :",font=labelfont,bg = 'lightblue')
        self.Joint3.grid(row=6, column=0, sticky=E)
        self.value3 = Entry(self.f1,font=labelfont)
        self.value3.grid(row=6, column=1,sticky=W)
        
        self.Joint4 = Label(self.f1, text="Joint4 :",font=labelfont,bg = 'lightblue')
        self.Joint4.grid(row=7, column=0, sticky=E)
        self.value4 = Entry(self.f1,font=labelfont)
        self.value4.grid(row=7, column=1, sticky=W)

        self.Joint5 = Label(self.f1, text="Joint5 :",font=labelfont,bg = 'lightblue')
        self.Joint5.grid(row=8, column=0, sticky=E)
        self.value5 = Entry(self.f1,font=labelfont)
        self.value5.grid(row=8, column=1, sticky=W)

        self.update = Button(self.f1,bg = 'lightblue', text="Update Joints value", command=self.update_joints,font=labelfont)
        self.update.grid(row=9, column=0, columnspan=2,ipadx=12)

        self.plan = Button(self.f1,bg = 'lightblue', text="Plan the trajectory", command=self.planning,font=labelfont)
        self.plan.grid(row=10, column=0, columnspan=2,ipadx=15)

        self.execute = Button(self.f1,bg = 'lightblue', text="Execute the trajectory", command=self.execution,font=labelfont)
        self.execute.grid(row=11, column=0, columnspan=2)

        self.save = Button(self.f1,bg = 'lightblue',text="save pose", command=self.save,font=labelfont)
        self.save.grid(row=12, column=0, columnspan=2,ipadx=50)
	
	self.label_error = Label(self.f1, text= " ") 
	self.label_error.grid(row=14,column=0,columnspan=2)   

        self.C1 = Checkbutton(self.f1,variable=num,command =self.enable_C1).grid(row=14,column=0,sticky=E)
        self.label_joy =Label(self.f1,bg = 'lightblue',text="Enable Joy-stick control",font=labelfont).grid(row=14,column=1,sticky = W)
        
        """self.slider1 = Scale(self.f1,state=DISABLED,orient=HORIZONTAL, length = 150, sliderlength = 10, from_= 4, to= 30,tickinterval=26)
        self.slider1.grid(row=15,column=1,sticky=W)"""
        
        #All that should be in frame 2
        self.boundary1 = Label(self.f2, bg = 'lightblue',text="=======================================",font=labelfont)
        #self.boundary1.grid(row=0, column=0,columnspan=2, sticky=W)
        self.label3 = Label(self.f2,bg = 'lightblue', text="Select one of the saved pose   ",font=labelfont)
        self.label3.grid(row=1, column=0, sticky=W)
        # this section is for making dropdown menu
        var, values = update_joints_from_txt()
        self.variable = StringVar(self.f2)
        self.variable.set(var[0])
        self.w = OptionMenu(self.f2,self.variable, *var)
        self.w.grid(row=1, column=2, sticky=E)
        self.w.config(bg = 'lightblue',font=labelfont)
        #self.w.grid_propagate(False)
        self.savedPoseExe = Button(self.f2,bg = 'lightblue', text="Execute selected pose",command=self.execution_saved,font=labelfont)
        self.savedPoseExe.grid(row=2, column=0,columnspan=2,ipadx=10,padx=50)
        self.deletePose = Button(self.f2,bg = 'lightblue', text="Delete selected pose",command=self.deleted_saved_pose,font=labelfont)
        self.deletePose.grid(row=3, column=0,columnspan=2,ipadx=15,padx=50)
        
        ##All that should be in frame 3
        group.set_end_effector_link('ee_link')
        ee_current = group.get_current_pose().pose
        ee_current_position = np.array([ee_current.position.x,ee_current.position.y,ee_current.position.z])
        self.label_ee_current=Label(self.f3,bg = 'lightblue', text="The current location of end effector is:[ %.3f, %.3f, %.3f]" %(ee_current_position[0],ee_current_position[1],ee_current_position[2]), anchor= 'w',font=labelfont)
        self.label_ee_current.grid(row=0,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
        self.label_wrist.grid(row=1,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        
        self.label4 = Label(self.f3,bg = 'lightblue', text="Please enter the desired position of wrist centre in meter: ",font=labelfont)
        self.label4.grid(row=2, column=0, columnspan=2, sticky=W)
        
        self.X_pos = Label(self.f3,bg = 'lightblue', text="X-Coordinate of wrist center :",font=labelfont)
        self.X_pos.grid(row=3, column=0, sticky=E)
        self.value6 = Entry(self.f3,font=labelfont)
        self.value6.grid(row=3, column=1, sticky=W)

        self.Y_pos = Label(self.f3,bg = 'lightblue', text="Y-Coordinate of wrist center :",font=labelfont)
        self.Y_pos.grid(row=4, column=0, sticky=E)
        self.value7 = Entry(self.f3,font=labelfont)
        self.value7.grid(row=4, column=1, sticky=W)

        self.Z_pos = Label(self.f3,bg = 'lightblue',text="Z-Coordinate of wrist center :",font=labelfont)
        self.Z_pos.grid(row=5, column=0, sticky=E)
        self.value8 = Entry(self.f3,font=labelfont)
        self.value8.grid(row=5, column=1,sticky=W)
        
        self.PTP= Button(self.f3,bg = 'lightblue', text="PTP,Move to above wrist position",command=self.perform_PTP,font=labelfont)
        self.PTP.grid(row=6, column=0, columnspan=2,ipadx=44)  
        
        self.LIN = Button(self.f3,bg = 'lightblue', text="LINEAR motion,Move to above wrist position",command=self.perform_LIN,font=labelfont)
        self.LIN.grid(row=7, column=0, columnspan=2) 
        
        #All that should be in frame 4
        self.boundary3 = Label(self.f4, bg = 'lightblue',text="=======================================",font=labelfont)
        #self.boundary3.grid(row=0, column=0,columnspan=2, sticky=W)
        self.label5 = Label(self.f4,bg = 'lightblue', text=" Enter the number of poses through which you want to move :     ",font=labelfont)
        self.label5.grid(row=1, column=0, columnspan=3, sticky=W)
        self.spinbox1 =Spinbox(self.f4, from_=1, to=10,font=labelfont)
        self.spinbox1.grid(row=2, column= 0,sticky=N)
        self.create = Button(self.f4,bg = 'lightblue', text="Create", command=self.create_selection_poses,font=labelfont)
        self.create.grid(row=2,column=1,sticky=W)
    
    def deleted_saved_pose(self):
        os.chdir("/home/pravin/catkin_ws")
        var, values = update_joints_from_txt()
        for i in var:
            if i ==self.variable.get():
                    pass
            else:
                with open('temp.txt','a') as f_temp:
                    pose_dict = {}
                    index_of_pose = var.index(i)
                    pose_dict[i]=values[index_of_pose]
                    f_temp.write(str(pose_dict)+"\n") 
        os.remove('posenames.txt')
        os.rename('temp.txt','posenames.txt')
        self.w.children["menu"].delete(0,"end")
        var, values = update_joints_from_txt()
        for v in var:
            self.w.children["menu"].add_command(label=v, command=lambda a = v: self.variable.set(a),font=labelfont)
        self.variable.set(var[0])
        global m,label_exe
        m+=1
        if (m > 1):
            #print m
            label_exe.destroy()
        label_exe= Label(self.f2,bg = 'lightblue', text= "Pose deleted",font=labelfont)
        label_exe.grid(columnspan=2)
                
                
    def enable_C1(self):
        global variable2,joy_part
        if num.get():
            self.label_sel_joy = Label(self.f1, bg = 'lightblue',text="Select motion",font=labelfont)
            self.label_sel_joy.grid(row=15,column=0,sticky =E)
            self.variable_joy = StringVar(self.f1)
            self.variable_joy .set("Allow stepwise motion")
            self.option_joy =OptionMenu(self.f1, self.variable_joy , "Allow stepwise motion","Allow continuous motion")
            self.option_joy.grid(row=15, column=1, sticky=W)
            self.option_joy.config(bg = 'lightblue',font=labelfont)
            self.joy_button = Button(self.f1,bg = 'lightblue', text="Activate selected motion", command=self.joy_activate,font=labelfont)
            self.joy_button.grid(row=16, column=0, columnspan=2)
        else:
            self.label_sel_joy.destroy()
            self.option_joy.destroy()
            self.joy_button.destroy()
            joy_part.unregister() # this will stop subscribing to joy so the joy-stick will no longer move
    
    
    #function to perform execution
    def execution1(self,joint_value):
        global group
        if joint_value[4] > 3.14:
            joint_value[4] = joint_value[4]-6.28
        group.set_joint_value_target(joint_value)
        plan2 = group.plan()
        execute2= group.execute(plan2)
        group_variable_values = group.get_current_joint_values()
        self.label1.destroy()
        self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
        self.label1.grid(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        self.label_wrist.destroy()
        self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
        self.label_wrist.grid(row=1,column = 0,columnspan=2)
        group.set_end_effector_link('ee_link')
        ee_current = group.get_current_pose().pose
        ee_current_position = np.array([ee_current.position.x,ee_current.position.y,ee_current.position.z])
        self.label_ee_current.destroy()
        self.label_ee_current=Label(self.f3,bg = 'lightblue', text="The current location of end effector is:[ %.3f, %.3f, %.3f]" %(ee_current_position[0],ee_current_position[1],ee_current_position[2]), anchor= 'w',font=labelfont)
        self.label_ee_current.grid(row=0,column = 0,columnspan=2)
    #function that are called after flag raised
    def robot_joy_inc(self,joint_value, joint_number, Limit1, Limit2):
        global group
        if (joint_value[joint_number-1] >= Limit1):
            if joint_value[joint_number-1] > 3.1:
                joint_value[joint_number-1] = -3.1 
            else:
                joint_value[joint_number-1] =  Limit1
            execution(joint_value)
            print "Limit reached joint %d" %joint_number
            print "Value of Joint {j_no} is: {j_val}".format(j_no=joint_number, j_val = joint_value[joint_number-1])
        elif (joint_value[joint_number-1] <= Limit2):
            if joint_value[joint_number-1] < -3.1:
                joint_value[joint_number-1] = 3.1 
            else:
                joint_value[joint_number-1] =  Limit2
            self.execution1(joint_value)
            print "Limit reached joint %d" %joint_number
            print "Value of Joint {j_no} is: {j_val}".format(j_no=joint_number, j_val = joint_value[joint_number-1])    
        else:
            print "changing joint %d" %joint_number
            print "Value of Joint {j_no} is: {j_val}".format(j_no=joint_number, j_val = joint_value[joint_number-1])
            print "plan complete"
            self.execution1(joint_value)

    #whenever there is data in callback we will be here
    def callback(self,data, selected_motion):
        global  j1, j2, j3, j4, j5, f1,f2,f3,f4,f5,flag1, flag2, flag3, flag4, flag5, flag6, flag7, flag8, flag9, flag10, group

        ########## Continous Movement ##########

        if (selected_motion == "Allow continuous motion"):
        ################ 0 & 6 #################
            if ((data.buttons[0]==1) and (data.axes[6]==1)):
                    j1 += 1
                    f1 = True
                    print j1
                    
            elif ((data.buttons[0]==1) and (data.axes[6]==-1)):
                    j1 -= 1
                    f1 = True
                    print j1
            
            else:
                #print "HI"
                if f1:
                    #print "HI"
                    f1 = False
                    group_variable_values = group.get_current_joint_values()
                    print group_variable_values
                    group_variable_values[0] += 0.01 * j1
                    j1= 0
                    self.robot_joy_inc(group_variable_values, 1, 3.1,-3.1)
                        
        ################ 1 & 6 #################
            if ((data.buttons[1]==1) and (data.axes[6]==1)):
                    j2 += 1
                    f2 = True
                    print j2
            elif ((data.buttons[1]==1) and (data.axes[6]==-1)):
                    j2 -= 1
                    f2 = True
                    print j2
            else:
                #print "HI1"
                if f2:
                    #print "HI1"
                    f2 = False
                    group_variable_values = group.get_current_joint_values()
                    print group_variable_values                
                    group_variable_values[1] += 0.01 * j2	
                    j2= 0
                    self.robot_joy_inc(group_variable_values, 2, 1.91,-1.22)
                    
        ################ 2 & 6 #################
            if ((data.buttons[2]==1) and (data.axes[6]==1)):
                    j3 += 1
                    f3 = True
                    print j3
            elif ((data.buttons[2]==1) and (data.axes[6]==-1)):
                j3 -= 1
                f3 = True
                print j3
            else:
                #print "HI2"
                if f3:
                    #print "HI2"
                    f3 =False
                    group_variable_values = group.get_current_joint_values()
                    print group_variable_values                
                    group_variable_values[2] += 0.01 * j3
                    j3= 0
                    self.robot_joy_inc(group_variable_values, 3, 0.95,-1.74)
                        
        ################ 3 & 6 #################
            if ((data.buttons[3]==1) and (data.axes[6]==1)):
                j4 += 1
                f4 = True
                print j4
            elif ((data.buttons[3]==1) and (data.axes[6]==-1)):
                j4 -= 1
                f4 = True
                print j4
            else:
                #print "HI3"
                if f4:
                    #print "HI3"
                    f4 =False
                    group_variable_values = group.get_current_joint_values()
                    print group_variable_values
                    group_variable_values[3] += 0.01 * j4
                    j4= 0
                    self.robot_joy_inc(group_variable_values, 4, 0.95,-1.83)
                    
        ################ 4 & 6 #################
            if ((data.buttons[4]==1) and (data.axes[6]==1)):
                j5 -= 1
                f5 = True
                print j5
            elif ((data.buttons[4]==1) and (data.axes[6]==-1)):
                j5 -= 1
                f5 = True
                print j5
            else:
                #print "HI4"
                if f5:
                    #print "HI3"
                    f5 = False
                    group_variable_values = group.get_current_joint_values()
                    print group_variable_values
                    group_variable_values[3] += 0.01 * j5
                    j5=0
                    self.robot_joy_inc(group_variable_values, 5, 3.1,-3.14)
                            
        ############### Single Step ############

        if (selected_motion== "Allow stepwise motion"):

        ################ 0 & 6 #################
                if ((data.buttons[0]==1) and (data.axes[6]==1)):
                    #print flag1
                    if flag1:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[0] = group_variable_values[0] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 1, 3.1,-3.1)
                    flag1 = False
                else:
                    #print flag1
                    flag1= True
                    
                if ((data.buttons[0]==1) and (data.axes[6]==-1)):
                    if flag1:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[0] = group_variable_values[0] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 1, 3.1,-3.1)
                    flag2 = False
                else:
                    flag2= True

        ################ 1 & 6 #################
                if ((data.buttons[1]==1) and (data.axes[6]==1)):
                    if flag3:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[1] = group_variable_values[1] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 2, 1.91,-1.22)
                    flag3 = False
                else:
                    flag3 = True
                
                if ((data.buttons[1]==1) and (data.axes[6]==-1)):
                    if flag4:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[1] = group_variable_values[1] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 2, 1.91,-1.22)
                    flag4 = False
                else:
                    flag4= True

        ################ 2 & 6 #################
                if ((data.buttons[2]==1) and (data.axes[6]==1)): 
                    if flag5:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[2] = group_variable_values[2] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 3, 0.95,-1.74)
                    flag5 = False
                else:
                    flag5 = True
                if ((data.buttons[2]==1) and (data.axes[6]==-1)):                
                    if flag6:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[2] = group_variable_values[2] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 3, 0.95,-1.74)
                    flag6 = False
                else:
                    flag6 = True		

        ################ 3 & 6 #################
                if ((data.buttons[3]==1) and (data.axes[6]==1)):  
                    if flag7:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[3] = group_variable_values[3] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 4, 0.95,-1.83)
                    flag7 = False
                else:
                    flag7 = True
                if ((data.buttons[3]==1) and (data.axes[6]==-1)):
                    if flag8:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[3] = group_variable_values[3] + 0.1*data.axes[6]                
                        self.robot_joy_inc(group_variable_values, 4, 0.95,-1.83)
                    flag8 = False
                else:
                    flag8 = True		

        ################ 4 & 6 #################
                if ((data.buttons[4]==1) and (data.axes[6]==1)):
                    if flag9:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[4] = group_variable_values[4] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 5, 3.1,-3.14)
                    flag9 = False
                else:
                    flag9 = True
                if ((data.buttons[4]==1) and (data.axes[6]==-1)):                
                    if flag10:
                        group_variable_values = group.get_current_joint_values()
                        print group_variable_values
                        group_variable_values[4] = group_variable_values[4] + 0.1*data.axes[6]
                        self.robot_joy_inc(group_variable_values, 5, 3.1,-3.14)
                    flag10 = False
                else:
                    flag10 = True

    
    def joy_activate(self):
        global joy_part
        #os.system('roslaunch igus_planning joy1.launch')
        joy_part =rospy.Subscriber("joy", Joy, self.callback, callback_args=self.variable_joy.get())
    
    # This function or method is used to calculate the pose in cartesian space of wrist centre from joint space. We also perform motion by calling linear_motion at the end
    def perform_FK(self,joint_values):
        # translation from world to base_link
        #print joint_values
        Wl1 = np.array([-0.06, 0, 0.1075])
        TWl1 = tran(Wl1)
        # DH parameters
        bita = np.array([0, 86.6, -92.6, 6, 0])
        d = np.array([0.110, 0, 0, 0, 0.19035])
        a = np.array([0, 0.270, 0.2403, 0, 0])
        alpha = np.array([90, 0, 0, 90, 0])
        
        # A1 represents transforms from frame 0 to 1
        theta1 = -np.rad2deg(joint_values[0]) + bita[0] # there is -ve because angles are taken clockwise but for calculation we use anticlockwise rotation as positive 
        #print theta1
        Rz1 = rotz(theta1)
        Tz1 = tran(np.array([0, 0, d[0]]))
        Tx1 = tran(np.array([a[0], 0, 0]))
        Rx1 = rotx(alpha[0])
        A11 = [Rx1, Tx1, Tz1, Rz1]
        A1 = matprod(A11)
        #print(A1)
        # A2 represents transforms from frame 1 to 2
        theta2 = -np.rad2deg(joint_values[1]) + bita[1]
        #print theta2
        Rz2 = rotz(theta2)
        Tz2 = tran(np.array([0, 0, d[1]]))
        Tx2 = tran(np.array([a[1], 0, 0]))
        Rx2= rotx(alpha[1])
        A22 = [Rx2, Tx2, Tz2, Rz2]
        A2 = matprod(A22)
        #print(A2)
        # A3 represents transforms from frame 2 to 3
        theta3 = -np.rad2deg(joint_values[2]) + bita[2]
        #print theta3
        Rz3 = rotz(theta3)
        Tz3 = tran(np.array([0, 0, d[2]]))
        Tx3 = tran(np.array([a[2], 0, 0]))
        Rx3 = rotx(alpha[1])
        A33 = [Rx3, Tx3, Tz3, Rz3]
        A3 = matprod(A33)
        #print(A3)

        T03 = [A3, A2, A1,TWl1]
        pos3 = matprod(T03)
        #print pos3
        X,Y,Z = pos3[0][3],pos3[1][3],pos3[2][3]
        self.linear_motion(X,Y,Z)
    
    # This method is used to perform trajectory motion as expected from fourth box in GUI
    def trajectory_motion (self):
        global variable1_list,variable2_list,group_variable_values,c1var
        print "trajectory_motion"
        traj = []
        for j in range(len(option_list)):
            #print variable1_list[j].get()
            var, values = update_joints_from_txt()
            index_of_pose = var.index(variable1_list[j].get())
            print(index_of_pose)
            print (type(group_variable_values))
            group_variable_values = (values[index_of_pose])
            if variable2_list[j].get() == 'PTP':
                group.set_joint_value_target(group_variable_values)
                group.go(wait=True)
            else:
                self.perform_FK(group_variable_values[0:3])
                group_variable_values= group.get_current_joint_values()
                group_variable_values[3] = values[index_of_pose][3]
                group_variable_values[4] = values[index_of_pose][4]
                group.set_joint_value_target(group_variable_values)
                group.go(wait=True)
            traj.append(group_variable_values)
        print c1var
        if c1var%2:
            print c1var
            for el in traj[::-1]: #we go in reverse order of traj
                group_variable_values = el
                if variable2_list[j].get() == 'PTP':
                    group.set_joint_value_target(group_variable_values)
                    group.go(wait=True)
                else:
                    self.perform_FK(group_variable_values[0:3])
                    group_variable_values= group.get_current_joint_values()
                    group_variable_values[3] = values[index_of_pose][3]
                    group_variable_values[4] = values[index_of_pose][4]
                    group.set_joint_value_target(group_variable_values)
                    group.go(wait=True)
        self.destroy_label1()
        
    def destroy_label1(self):        
        self.label1.destroy() 
        self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
        self.label1.grid(row=0, column=0, columnspan=2, sticky=W)
        #print traj
        
    """def loop_forever(self):
        global loop_flag,c1var,root
        if loop_flag:
            c1var = 1
            self.trajectory_motion()
            root.after(10000, self.loop_forever()) #wait for 2 sec then call loop again
    #This method is used to create mutliple selection option based on user entry. Old labels and optionmenus are destroyed""" 
    def create_selection_poses(self):
        global previous_input, label_list, option_list,option2_list,trajectory1,variable1_list,variable2_list
        num_pose_to_create = self.spinbox1.get()

            #print num_pose_to_create
        try:
            if previous_input > 0:
                #print label_list
                for j in range(len(label_list)):
                    label_list[j].destroy()
                    option_list[j].destroy()
                    option2_list[j].destroy()
                trajectory1.destroy()
                self.Check1.destroy()
                self.label6.destroy()
                #self.label7.destroy()
                #self.label8.destroy()
            if int(num_pose_to_create)<11 and int(num_pose_to_create) >0:
                option_list= []
                label_list = []
                variable1_list=[]
                variable2_list=[]
                option2_list= []
                motion_type =["PTP","LIN"]
                for i in range(int(num_pose_to_create)):
                    #print i
                    Pose1 = Label(self.f4,bg = 'lightblue', text="Select pose number %d " %int(i+1),font=labelfont)
                    label_list.append(Pose1)
                    Pose1.grid(row=i+3,column=0,sticky=E)
                    var, values = update_joints_from_txt()
                    variable1 = StringVar(self.f4)
                    variable1_list.append(variable1)
                    variable1.set(var[0])
                    option1 = OptionMenu(self.f4, variable1, *var)
                    option1.grid(row=i+3, column=1, sticky=W)
                    option_list.append(option1)
                    option1.config(bg = 'lightblue',font=labelfont)
                    variable2 = StringVar(self.f4)
                    variable2_list.append(variable2)
                    variable2.set("PTP")
                    option2 =OptionMenu(self.f4, variable2, "PTP","LIN")
                    option2.grid(row=i+3, column=2, sticky=W)
                    option2_list.append(option2)
                    option2.config(bg = 'lightblue',font=labelfont)
                    
                previous_input = i+1
                #print previous_input
                #print label_list
                trajectory1 =Button(self.f4, bg = 'lightblue',text="Move to selected poses", command=self.trajectory_motion,font=labelfont)
                trajectory1.grid(row=i+5, column=1, columnspan=2,ipadx=10)
                self.Check1 = Checkbutton(self.f4,command=self.enable, variable=c1var)
                self.Check1.grid(row=i+4,column=0,sticky=E)
                self.label6 = Label(self.f4,bg = 'lightblue', text=" Return to pose number 1",font=labelfont)
                self.label6.grid(row=i+4, column=1, columnspan=2, sticky=W)
                #self.label7 = Button(self.f4,bg = 'lightblue', text=" Loop forever",font=labelfont,command=self.loop_forever)
                #self.label7.grid(row=i+7, column=1, columnspan=2,ipadx=50)
                #self.label8 = Button(self.f4,bg = 'lightblue', text=" Stop loop",font=labelfont,command=self.stop_loop)
                #self.label8.grid(row=i+8, column=1, columnspan=2,ipadx=65)
            else:
                print("Please enter value between 1 and 10")
        except:
            pass
    
    """def stop_loop(self):
        global loop_flag
        loop_flag = False
        self.loop_forever()
        loop_flag = False"""
    #this enables the checkbox
    def enable(self):
        global c1var
        c1var = c1var+1

    #this is responsible only for planning.Actual motion is not performed here        
    def planning(self):
        global n
        # n is basically used to check if some error or msg is already printed. If present we clear the msg and start a new one
        n+=1
        if (n > 1):
            print n
            self.label_error.destroy()
        self.label_error= Label(self.f1,bg = 'lightblue', text= "Planning done",font=labelfont)
        self.label_error.grid(columnspan=2)
        # setting the joints value for planning
        group.set_joint_value_target(group_variable_values)
        # plan instruction
        group.plan()
        #wait for 1 sec
        rospy.sleep(1)
    
    # here we update the joint with new values that are obtained from user data
    def update_joints(self):
        try:
            global n
            # we check whether the entered value are within the limits of joints based on urdf file
            if (float(self.value1.get()) > -180) and (float(self.value1.get())<180):
                j1 = float(self.value1.get())
                group_variable_values[0] = np.deg2rad(j1)#convert to rad
                print group_variable_values[0]
            else:
                n=n+1
                #print n
                if (n > 1):
                    print n
                    self.label_error.destroy()
                print("Enter value between -180 and 180")
                self.label_error = Label(self.f1, bg = 'lightblue',text= "Enter value between -180 and 180 in Joint1",font=labelfont)
                self.label_error.grid(columnspan=2)
            if (float(self.value2.get()) > -69.96) and (float(self.value2.get())<109.94):
                j2 = float(self.value2.get())
                group_variable_values[1] = np.deg2rad(j2)
                print group_variable_values[1]
                
            else:
                n=n+1
                #print n
                if (n > 1):
                    #print n
                    self.label_error.destroy()
                print("Enter value between -69 and 109")
                self.label_error = Label(self.f1,bg = 'lightblue', text= "Enter value between -69 and 109 in Joint2",font=labelfont)
                self.label_error.grid(columnspan=2)
            if (float(self.value3.get()) > -99.94) and (float(self.value3.get())<54.97):
                j3 = float(self.value3.get())
                group_variable_values[2] = np.deg2rad(j3)
                print group_variable_values[2]
            else:
                n=n+1
                #print n
                if (n > 1):
                    #print n
                    self.label_error.destroy()
                print("Enter value between -99 and 54")
                self.label_error = Label(self.f1,bg = 'lightblue', text= "Enter value between -99 and 54 in Joint3",font=labelfont)
                self.label_error.grid(columnspan=2)
            if (float(self.value4.get()) > -104.94) and (float(self.value4.get())<54.97):
                j4 = float(self.value4.get())
                group_variable_values[3] = np.deg2rad(j4)
                print group_variable_values[3]
            else:
                n=n+1
                #print n
                if (n > 1):
                    #print n
                    self.label_error.destroy()
                print("Enter value between -104 and 54")
                self.label_error = Label(self.f1,bg = 'lightblue', text= "Enter value between -104 and 54 in Joint4",font=labelfont)
                self.label_error.grid(columnspan=2)
            if (float(self.value5.get()) > -180) and (float(self.value5.get())<180):
                j5 = float(self.value5.get())
                group_variable_values[4] = np.deg2rad(j5)
                print group_variable_values[4]
            else:
                n=n+1
                #print n
                if (n > 1):
                    #print n
                    self.label_error.destroy()
                print("Enter value between -180 and 180")
                self.label_error = Label(self.f1,bg = 'lightblue', text= "Enter value between -180 and 180 in Joint",font=labelfont)
                self.label_error.grid(columnspan=2)
            n=n+1
            #print n
            if (n > 1):
                #print n
                self.label_error.destroy()
            self.label_error = Label(self.f1, bg = 'lightblue',text= "joints values updated, now you can plan",font=labelfont)
            self.label_error.grid(columnspan=2)
        #if any problem occurs above, we come to this section       
        except:
            global n
            n=n+1
            #print n
            if (n > 1):
                #print n
                self.label_error.destroy()
            self.label_error= Label(self.f1,bg = 'lightblue', text= "ERROR ! : Enter correct values in all boxes",font=labelfont)
            self.label_error.grid(columnspan=2)
        print group_variable_values
        return group_variable_values,n
 
    # execution is performed based on plan performed on planning function above
    def execution(self):
        global n
        n+=1
        if (n > 1):
            #print n
            self.label_error.destroy()
        self.label_error= Label(self.f1,bg = 'lightblue', text= "execution done",font=labelfont)
        self.label_error.grid(columnspan=2)
        print("execution done")
        #execute instruction
        group.go(wait=True)
        # wait for 1 sec
        rospy.sleep(1)
        self.label1.destroy()
        self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
        self.label1.grid(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        self.label_wrist.destroy()
        self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
        self.label_wrist.grid(row=1,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('ee_link')
        ee_current = group.get_current_pose().pose
        ee_current_position = np.array([ee_current.position.x,ee_current.position.y,ee_current.position.z])
        self.label_ee_current.destroy()
        self.label_ee_current=Label(self.f3,bg = 'lightblue', text="The current location of end effector is:[ %.3f, %.3f, %.3f]" %(ee_current_position[0],ee_current_position[1],ee_current_position[2]), anchor= 'w',font=labelfont)
        self.label_ee_current.grid(row=0,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        #print n

    # here we execute the saved poses
    def execution_saved(self):
        global group,group_variable_values, m,label_exe
        m+=1
        if (m > 1):
            #print m
            label_exe.destroy()
        label_exe= Label(self.f2,bg = 'lightblue', text= "execution done",font=labelfont)
        label_exe.grid(columnspan=2)
        print("execution done")
        var, values = update_joints_from_txt()
        index_of_pose = var.index(self.variable.get())
        print(index_of_pose)
        print (type(group_variable_values))
        group_variable_values = (values[index_of_pose])
            #print group_variable_values
            #print (type(group_variable_values[0]))
        group.set_joint_value_target(group_variable_values)
        group.plan()
        #rospy.sleep(1)
        group.go(wait=True)
        #rospy.sleep(1)
        group_variable_values= group.get_current_joint_values()
        self.label1.destroy()
        label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
        label1.grid(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        self.label_wrist.destroy()
        self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
        self.label_wrist.grid(row=1,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('ee_link')
        ee_current = group.get_current_pose().pose
        ee_current_position = np.array([ee_current.position.x,ee_current.position.y,ee_current.position.z])
        self.label_ee_current.destroy()
        self.label_ee_current=Label(self.f3,bg = 'lightblue', text="The current location of end effector is:[ %.3f, %.3f, %.3f]" %(ee_current_position[0],ee_current_position[1],ee_current_position[2]), anchor= 'w',font=labelfont)
        self.label_ee_current.grid(row=0,column = 0,columnspan=2)
    # this function is responsible to save the current pose of joints. It calls the popupwindow class.
    def save(self):
        # we save based on the current joints value 
        #print("Please name sth")
        self.new = popupwindow(self.master)
        self.master.wait_window(self.new.top)
        #print(self.new.value)
        # this part helps to make the gui update the dropdown menu if new value is added
        #temp=[]
        self.w.children["menu"].delete(0,"end")
        var, values = update_joints_from_txt()
        """with open('posenames.txt','r') as f11:
            comments=f11.readlines()
    	    var =[]
    	    values =[]
    	    for i in comments:
        	a = i.strip()
                if a != '':
                    a1 = eval(a)
                for key in a1:
                    var.append(key)
                    values.append(a1[key])
            #print var"""

            #self.w = OptionMenu(self.f2, self.variable, *var)
            #print temp
        for v in var:
            self.w.children["menu"].add_command(label=v, command=lambda a = v: self.variable.set(a),font=labelfont)
        self.variable.set(var[0])
    
    # Get the input from the entry
    def get_wrist_pose(self):
        try:
            global o, label_error3
            X = float(self.value6.get())
            Y = float(self.value7.get())
            Z = float(self.value8.get())
            return X,Y,Z
        except:
            o=o+1
            #print n
            if (o > 1):
                print o
                label_error3.destroy()
            label_error3= Label(self.f3,bg = 'lightblue', text= "ERROR ! : Enter correct values in all boxes",font=labelfont)
            label_error3.grid(columnspan=2)
    
    # it performs inverse kinematics
    def perform_IK(self,X, Y, Z):
        try:
            global  o, label_error3,group_variable_values, group
            print (X,Y,Z)
            o=o+1
            if (o > 1):
                label_error3.destroy()
            label_error3= Label(self.f3, bg = 'lightblue',text= "IK solutions found",font=labelfont)
            label_error3.grid(columnspan=2) 
            # from urdf file, moving to frame j2, regardless of orientation
            Wl2 = np.array([-0.06, 0, 0.2175]) # i have added z=0.110 to world to frame1 ,as frame 2 is directly up from frame1 at a distance of 0.110
            # These are the distance from the wrist centre to the origin of frame 2
            x_new = X - Wl2[0]
            y_new = Y - Wl2[1]
            z_new = Z - Wl2[2]
            print(x_new,y_new,z_new)
            
            #Assuming that the position that could be reached by the arm is a position within or on the surface of sphere. We check distance from the frame centre to (x_new,y_new,z_new) to be less than or equal to R
            # some parameters from DH-Table
            a = np.array([0, 0.270, 0.2403, 0, 0])
            bita = np.array([0, 86.6, -92.6, 6, 0])
            R = a[1] + a[2]  # These are the link lenghts taken from DH parameters 
            
            if (np.sqrt(x_new ** 2 + y_new ** 2 + z_new ** 2) > R):
                o=o+1
                if (o > 1):
                    label_error3.destroy()
                label_error3= Label(self.f3,bg = 'lightblue', text= "Position entered cannot be reached",font=labelfont)
                label_error3.grid(columnspan=2)
            else:
                global group
                print("Position entered could be reached")
                thita1 = np.arctan2(y_new, x_new)*180/np.pi
                thita11 = np.arctan2(-y_new, -x_new)*180/np.pi
                # i have copied many files from inverse_kinematic_server.py
                #please refer to that file for further assistance
                r = np.sqrt(y_new * y_new + x_new * x_new)
                h = z_new
                psi = np.arctan2(h, r)*180 / np.pi
                c = np.sqrt(r ** 2 + h ** 2)
                D1 = (a[1] ** 2 + c ** 2 - a[2] ** 2) / (2 * a[1] * c)
                D2 = np.sqrt(1-D1*D1)
                thita2 = -(psi-bita[1]+(np.arctan2(D2, D1))*180/np.pi)
                thita22 = 180-psi-bita[1]+(np.arctan2(D2, D1))*180/np.pi
                D3 = (a[1] ** 2 - c ** 2 + a[2] ** 2) / (2 * a[1] * a[2])
                D4 = D2 = np.sqrt(1-D3*D3)
                thita3 = -np.arctan2(D4, D3)*180/np.pi+bita[2]+180
                print(thita1,thita11,thita2,thita22,thita3)
                group_variable_values= group.get_current_joint_values()
                print(group_variable_values)
                group_variable_values[0] = np.deg2rad(thita1)
                group_variable_values[1] = np.deg2rad(thita2)
                group_variable_values[2] = np.deg2rad(thita3)
                group_variable_values[4] = 0 # i have added this because the joint5 value is going beyond 180 and -180
                print(group_variable_values)
                print(type(group_variable_values))
                group.set_joint_value_target(group_variable_values)
                group.plan()
                
                #rospy.sleep(1)
                group.go(wait=True)
                return group_variable_values
                #rospy.sleep(1)
                """group.set_end_effector_link('l4')
                wrist_current = group.get_current_pose().pose
                wrist_current_position = np.array([round(wrist_current.position.x,2),round(wrist_current.position.y,2),round(wrist_current.position.z,2)])
                print wrist_current_position
                print np.array([X,Y,Z])
                if np.array_equal(wrist_current_position, np.array([X,Y,Z])):
                    print ("All good. IK solution found")
                    #pass
                
                else:
                    print("Something is wrong. Check console for other information")
                    o=o+1
                    if (o > 1):
                        label_error3.destroy()
                    label_error3= Label(self.f3, text= "Something is wrong. Check console for other information",font=labelfont)
                    label_error3.grid(columnspan=2)"""
                    
        #if any problem occurs above, we come to this section       
        except:
            o=o+1
            if (o > 1):
                label_error3.destroy()
            label_error3= Label(self.f3, text= "Couldnot perfom Inverse Kinematics",font=labelfont)
            label_error3.grid(columnspan=2)
            pass
        #print (X,Y,Z)
    
    #Point to point motion is carried out here
    def perform_PTP(self):
        try:
            X,Y,Z = self.get_wrist_pose()
            print (X,Y,Z)
            self.perform_IK(X,Y,Z)
            self.label1.destroy()
            self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
            self.label1.grid(row=0, column=0, columnspan=2, sticky=W)
            group.set_end_effector_link('l4')
            wrist_current = group.get_current_pose().pose
            wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
            self.label_wrist.destroy()
            self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
            self.label_wrist.grid(row=0,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
        except:
            pass

    #we do linear motion here. Interpolation is carried out to get the points on linear path. We perform IK after that.
    def linear_motion(self,X,Y,Z):
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        print wrist_current_position
        #requested pose
        
        print (X,Y,Z)
        # step size. I am assuming 10 cm
        step = 0.01
        # difference between current and requested pose
        v = np.array([X - wrist_current_position[0], Y - wrist_current_position[1], Z - wrist_current_position[2]])
        #print v
        # distance between those two points or norm of v
        dist_v = np.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2 )
        
        x1 = np.arange(0, dist_v+step, step)
        #print x1
        #save_for_uc = []
        for x in x1:
            v1 =  wrist_current_position + v*x/dist_v
            print v1
            angles1 =self.perform_IK(v1[0],v1[1],v1[2])
            angles1 = [round(Decimal(i *180/np.pi), 2) for i in angles1] 
            angles2 = [round(i*100) for i in angles1]
            #save_for_uc.append(angles1)
            #if we want to save these angles in some format
            '''with open('uc.txt', 'a') as f_uc:
                f_uc.write('%s\n' %angles1)
            # we write this part for serial communication, if it could be added in future work. Each pose begins with S and ends with E.
            # For each angle we will have 5 character, 1 extra character to indicate the sign of this angle
            with open('uc_serial', 'a') as f_us:
                f_us.write('S')
                for i in angles2:
                    if i >=0:
                       f_us.write('0') 
                    else:
                        i = abs(i)
                        f_us.write('1')
                    i = str(int(i))
                    t = i.rjust(5,'0')
                    f_us.write(t)
                f_us.write('E')'''
                
        #print save_for_uc
        
    # we just call the above function here. The reason we separate the above function from this block is because it is also called by  trajectory_motion function
    def perform_LIN(self):
        # getting current pose of wrist centre. center of frame l4 is our wrist centre
        X,Y,Z = self.get_wrist_pose()
        self.linear_motion(X,Y,Z)
        self.label1.destroy()
        self.label1=Label(self.f1,bg = 'lightblue', text="The current Pose of the robot is:[ %.2f, %.2f, %.2f, %.2f, %.2f ]" %(group_variable_values[0],group_variable_values[1],group_variable_values[2],group_variable_values[3],group_variable_values[4]),font=labelfont)
        self.label1.grid(row=0, column=0, columnspan=2, sticky=W)
        group.set_end_effector_link('l4')
        wrist_current = group.get_current_pose().pose
        wrist_current_position = np.array([wrist_current.position.x,wrist_current.position.y,wrist_current.position.z])
        self.label_wrist.destroy()
        self.label_wrist=Label(self.f3,bg = 'lightblue', text="The current location of wrist center is:[ %.3f, %.3f, %.3f]" %(wrist_current_position[0],wrist_current_position[1],wrist_current_position[2]), anchor= 'w',font=labelfont)
        self.label_wrist.grid(row=0,column = 0,columnspan=2)#(row=0, column=0, columnspan=2, sticky=W)
            
                               


def move_group_python_interface():
    moveit_commander.roscpp_initialize(sys.argv) #initialising the moveit_commander
    rospy.init_node('move_group_python_interface', anonymous=True) # start the node 

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    global group, root 
    group= moveit_commander.MoveGroupCommander("igus_planning")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    global group_variable_values,n,m,o,previous_input,label_list,elements_list, trajectory1,variable1,variable2, joy_part, loop_flag#, value1,value2, value3,value4,value5 ,label1,n
    #joy_part is used to control subscription to joy topic
    # these global variable will be used to delete old labels
    n = 0
    m = 0
    o = 0
    previous_input = 0
    label_list=[]
    option_list=[]
    option2_list=[]
    loop_flag= True
    group_variable_values= group.get_current_joint_values()
    root = Tk()
    root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))
    root.configure(background = 'white')
    root.title("IGUS KINEMATICS")
    convas1 = Canvas(root,height=200, width= 400)
    convas1.place(x=0, y=0)
    my_image1= PhotoImage(file="/home/pravin/Desktop/IGUS_LOGO.png")
    convas1.create_image(0,0, anchor= NW, image = my_image1)
    convas2 = Canvas(root,height=200, width= 320)
    convas2.place(x=1400,y = 0)
    lbl = Label(root, text="KINEMATICS", font=("Helvetica", 110), bg='white')
    lbl.place(x = 440, y =0)
    my_image2= PhotoImage(file="/home/pravin/Desktop/rst.png")
    convas2.create_image(0,0, anchor= NW, image = my_image2)
    fk =gui(root)
    root.mainloop()

if __name__=='__main__':
  try:
    move_group_python_interface()
  except rospy.ROSInterruptException:
    pass

