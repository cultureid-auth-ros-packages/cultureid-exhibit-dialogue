#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
import os
import numpy as np
import time
import copy
from subprocess import call, Popen
from functools import partial
import random
import hashlib
import Tkinter
import tkFont
from PIL import Image,ImageTk

# Import ROS modules
import rospy
import tf

import roslib
roslib.load_manifest("rosparam")
import rosparam

import exhibit_dialogue

################################################################################
class ExhibitDialogueMain():

  ##############################################################################
  # constructor
  ##############################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)

    # Load params for this class
    self.init_params()

    # Let's go
    self.choose_exhibit_screen()

    # DJ...SPIN THAT SHIT
    self.root.mainloop()


  ##############################################################################
  def get_x_y_dims(self,desiredNum):

    side1 = int(desiredNum/int(desiredNum**0.5))
    side2 = int(desiredNum/side1)

    sideSmall = np.min([side1,side2])
    sideLarge = np.max([side1,side2])

    if sideSmall*sideLarge < desiredNum:
      sideSmall = sideSmall+1

    side1 = np.min([sideSmall,sideLarge])
    side2 = np.max([sideSmall,sideLarge])

    return side1,side2


  ##############################################################################
  def choose_exhibit_screen(self):
    rospy.logwarn('[%s] main; init', self.pkg_name)

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)

    q_button_vec = []
    q_button_txt = []

    # START BUTTONS ------------------------------------------------------------
    QButton = Tkinter.Button(self.frame_,text='???',fg='#E0B548',bg='white')
    q_button_vec.append(QButton)
    q_button_txt.append('ΕΙΣΑΓΕΤΕ ΤΟ ΟΝΟΜΑ ΤΟΥ ΕΚΘΕΜΑΤΟΣ')

    QButton = Tkinter.Button(self.frame_,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548', command=partial(self.process_exhibit_pressed,0))
    q_button_vec.append(QButton)
    q_button_txt.append('ΧΡΥΣΟΣ ΤΩΝ ΜΑΚΕΔΟΝΩΝ')

    QButton = Tkinter.Button(self.frame_,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548', command=partial(self.process_exhibit_pressed,1))
    q_button_vec.append(QButton)
    q_button_txt.append('ΠΑΠΥΡΟΣ ΤΟΥ ΔΕΡΒΕΝΙΟΥ')



    xNum = 1
    yNum = len(q_button_vec)


    xEff = 1.0
    yEff = 0.7
    GP = 0.1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):
        thisX = xG/2+xx*xWithGuard
        thisY = yG/2+yy*yWithGuard+0.1

        q_button_vec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        q_button_vec[counter].config(text=q_button_txt[counter])
        q_button_vec[counter].update()

        thisWidth = q_button_vec[counter].winfo_width()
        thisHeight = q_button_vec[counter].winfo_height()

        q_button_vec[counter].config(font=("Helvetica", 30))
        q_button_vec[counter].update()

        counter = counter+1






    # KILL APP BUTTON --------------------------------------------------------------
    this_butt = Tkinter.Button(self.frame_,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40', \
        command=self.kill_root)
    a_button_vec = []
    a_button_vec.append(this_butt)

    a_button_txt = []
    a_button_txt.append('ΕΞΟΔΟΣ')

    xNum,yNum = self.get_x_y_dims(len(a_button_vec))

    xEff = 1.0
    yEff = 0.2
    GP = 0.1

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = 0
    for xx in range(xNum):
      for yy in range(yNum):

        if counter < len(a_button_vec):
          thisX = xG/2+xx*xWithGuard
          thisY = yG/2+yy*yWithGuard+1-yEff

          a_button_vec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          a_button_vec[counter].config(text=a_button_txt[counter])
          a_button_vec[counter].update()

          thisWidth = a_button_vec[counter].winfo_width()
          thisHeight = a_button_vec[counter].winfo_height()
          a_button_vec[counter].config(font=("Helvetica", 24))
          a_button_vec[counter].config(wraplength=thisWidth-10,justify="center")
          a_button_vec[counter].update()

        counter = counter+1


  ##############################################################################
  def init_params(self):

    self.pkg_name = rospy.get_param('~pkg_name', '')
    self.pkg_ap = rospy.get_param('~pkg_ap', '')
    self.exhibit_titles = rospy.get_param('~exhibit_titles', '')
    self.exhibit_codes = rospy.get_param('~exhibit_codes', '')
    self.listening_imagefile = rospy.get_param('~listening_imagefile', '')
    self.speaking_imagefile = rospy.get_param('~speaking_imagefile', '')
    self.navigation_warnfile = rospy.get_param('~navigation_warnfile', '')
    self.navigation_audiofile = rospy.get_param('~navigation_audiofile', '')
    self.navigation_imagefile = rospy.get_param('~navigation_imagefile', '')
    self.transcript_file_path = rospy.get_param('~transcript_file_path', '')
    self.transcript_file_readrate = rospy.get_param('~transcript_file_readrate', '')
    self.rasa_upfile = rospy.get_param('~rasa_upfile', '')
    self.rasa_downfile = rospy.get_param('~rasa_downfile', '')
    self.s2s_upfile = rospy.get_param('~s2s_upfile', '')
    self.s2s_downfile = rospy.get_param('~s2s_downfile', '')


    if self.pkg_name == '':
      rospy.logerr('ExhibitDialogue pkg_name not set; aborting')
      return

    if self.pkg_ap == '':
      rospy.logerr('ExhibitDialogue pkg_ap not set; aborting')
      return

    if self.exhibit_titles == '':
      rospy.logerr('[%s] exhibit_titles not set; aborting', self.pkg_name)
      return

    if self.exhibit_codes == '':
      rospy.logerr('[%s] exhibit_codes not set; aborting', self.pkg_name)
      return

    if self.listening_imagefile == '':
      rospy.logerr('[%s] listening_imagefile not set; aborting', self.pkg_name)
      return

    if self.speaking_imagefile == '':
      rospy.logerr('[%s] speaking_imagefile not set; aborting', self.pkg_name)
      return

    if self.navigation_warnfile == '':
      rospy.logerr('[%s] navigation_warnfile not set; aborting', self.pkg_name)
      return

    if self.navigation_audiofile == '':
      rospy.logerr('[%s] navigation_audiofile not set; aborting', self.pkg_name)
      return

    if self.navigation_imagefile == '':
      rospy.logerr('[%s] navigation_imagefile not set; aborting', self.pkg_name)
      return

    if self.transcript_file_path == '':
      rospy.logerr('[%s] transcript_file_path not set; aborting', self.pkg_name)
      return

    if self.transcript_file_readrate == '':
      rospy.logerr('[%s] transcript_file_readrate not set; aborting', self.pkg_name)
      return

    if self.rasa_upfile== '':
      rospy.logerr('[%s] rasa_upfile not set; aborting', self.pkg_name)
      return
    if self.rasa_downfile== '':
      rospy.logerr('[%s] rasa_downfile not set; aborting', self.pkg_name)
      return
    if self.s2s_upfile== '':
      rospy.logerr('[%s] s2s_upfile not set; aborting', self.pkg_name)
      return
    if self.s2s_downfile== '':
      rospy.logerr('[%s] s2s_downfile not set; aborting', self.pkg_name)
      return



  ##############################################################################
  def kill_root(self):
    rospy.logerr('[%s] main; App killed. See you next time', self.pkg_name)

    call(['bash', '/home/cultureid_user0/game_desktop_launchers/kill_all.sh'])
    self.root.destroy()
    os._exit(os.EX_OK)


  ##############################################################################
  def load_exhibit_params(self, exhibit_id):

    # Load these variables
    self.tl_variables = rospy.get_param('~tl_variables', '')

    # Load the variables inside these .yaml files
    self.tl_files = rospy.get_param('~tl_files', '')

    # Load these variables
    self.tl_common_files = rospy.get_param('~tl_common_files', '')

    if self.tl_variables == '':
      rospy.logerr('[%s] tl_variables not set; aborting', self.pkg_name)
      return

    if self.tl_files == '':
      rospy.logerr('[%s] tl_files not set; aborting', self.pkg_name)
      return

    if self.tl_common_files == '':
      rospy.logerr('[%s] tl_common_files not set; aborting', self.pkg_name)
      return

    # Start loading params necessary for exhibit exhibit_id
    ns = self.pkg_name + '_node/'

    for i in range(len(self.tl_variables)):
      rospy.loginfo('Setting param %s to %s', \
          ns + self.tl_variables[i][0], \
          self.pkg_ap + self.tl_variables[i][1] + exhibit_id)
      rospy.set_param(\
          ns + self.tl_variables[i][0], \
          self.pkg_ap + self.tl_variables[i][1] + exhibit_id)

    for i in range(len(self.tl_files)):
      self.load_params_from_yaml(\
          self.pkg_ap + self.tl_files[i] + exhibit_id + '.yaml', ns)

    for i in range(len(self.tl_common_files)):
      rospy.loginfo('Setting param %s to %s', \
          ns + self.tl_common_files[i][0], \
          self.pkg_ap + self.tl_common_files[i][1])
      rospy.set_param(\
          ns + self.tl_common_files[i][0], \
          self.pkg_ap + self.tl_common_files[i][1])


  ##############################################################################
  def load_params_from_yaml(self, file_path, myns):

    paramlist = \
        rosparam.load_file(file_path,default_namespace=myns)
    for params, ns in paramlist:
      rosparam.upload_params(ns,params)


  ##############################################################################
  def new_canvas(self):
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='white')
    self.canvas_ = canvas
    return canvas


  ##############################################################################
  def new_frame(self):

    # clean window
    for frames in self.root.winfo_children():
      frames.destroy()

    frame = Tkinter.Frame(self.root,bg='white')
    frame.place(relwidth=0.95,relheight=0.95,relx=0.025,rely=0.025)
    self.frame_ = frame

    return frame


  ##############################################################################
  def process_exhibit_pressed(self, exhibit_id):

    # Init rasa stuff
    call(['bash', self.rasa_upfile])

    # should have arg exhibit_id for different rasa models (depending on exhibit)
    ed = exhibit_dialogue.ExhibitDialogue()

    # play again?
    self.choose_exhibit_screen()


  ##############################################################################
  def set_canvas(self, canvas):
    self.canvas_ = canvas


  ##############################################################################
  def set_frame(self, frame):
    self.frame_ = frame



################################################################################
# main
################################################################################
if __name__ == '__main__':

  rospy.init_node('amth_exhibit_dialogue_main_node')

  try:
    ed = ExhibitDialogueMain()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
