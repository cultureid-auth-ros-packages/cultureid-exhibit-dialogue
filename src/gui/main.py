#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import threading
import os
import numpy as np
import time
from subprocess import call, Popen
from functools import partial
import random
import Tkinter
import tkFont
from PIL import Image,ImageTk

# Import ROS modules
import rospy
import tf

# Sending goals etc
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

from std_msgs.msg import Empty
from std_srvs.srv import Empty as srv_empty
from std_srvs.srv import EmptyRequest as srv_empty_req
from std_msgs.msg import Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Vector3

import roslib
roslib.load_manifest("rosparam")
import rosparam



class ExhibitDialogue():

  ##############################################################################
  # constructor
  ##############################################################################
  def __init__(self):

    self.root = Tkinter.Tk()
    self.root.attributes('-fullscreen',True)

    # Sets the pose estimate of amcl
    self.amcl_init_pose_pub = \
        rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)

    # Publishes raw velocity commands to the wheels of the base
    # [for initial pose calibration]
    self.raw_velocity_commands_pub = \
        rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Load params for this class
    self.init_params()

    # Read transcript file every `read_rate` seconds
    tf_read_rate = rospy.Duration(1.0 / self.transcript_file_readrate)
    rospy.Timer(tf_read_rate, self.read_transcript_file)

    # When starting-up reset the transcript file
    self.reset_file(self.transcript_file_path)

    # Let's go
    self.main_screen()

    # DJ...SPIN THAT SHIT
    self.root.mainloop()



  ##############################################################################
  def display_message(self, message):

    frame = self.new_frame()
    self.set_frame(frame)

    # ta koumpia tou para8urou
    buttonVec = []
    buttonText = []

    playButton = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    buttonVec.append(playButton)
    buttonText.append(message)

    xNum = 1
    yNum = len(buttonVec)

    xEff = 1
    yEff = 1

    GP = 0.05

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
        thisY = yG/2+yy*yWithGuard

        buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        buttonVec[counter].config(text=buttonText[counter])
        buttonVec[counter].update()

        thisWidth = buttonVec[counter].winfo_width()
        thisHeight = buttonVec[counter].winfo_height()
        buttonVec[counter].config(font=("Helvetica", 20))
        buttonVec[counter].update()

        counter = counter+1


  ##############################################################################
  # exit button
  def exit_button(self,frame):

    buttonVec = []
    buttonText = []

    this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='red',activeforeground='#E0B548',activebackground='#343A40', command=self.kill_root)

    buttonVec.append(this_butt)
    this_group_name = 'X'
    buttonText.append(this_group_name)

    xNum = 1
    yNum = 1

    xEff = 0.075
    yEff = 0.075

    GP = 0.175

    xWithGuard = xEff/xNum
    xG = GP*xWithGuard
    xB = xWithGuard-xG

    yWithGuard = yEff/yNum
    yG = GP*yWithGuard
    yB = yWithGuard-yG

    counter = len(buttonVec)-1
    for xx in range(xNum):
      for yy in range(yNum):

        if counter < len(buttonVec):
          thisX = xG/2+xx*xWithGuard+1-xEff
          thisY = yG/2+yy*yWithGuard

          buttonVec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          buttonVec[counter].config(text=buttonText[counter])
          buttonVec[counter].update()

          thisWidth = buttonVec[counter].winfo_width()
          thisHeight = buttonVec[counter].winfo_height()
          buttonVec[counter].config(font=("Helvetica", 20))
          buttonVec[counter].update()

        counter = counter+1

    return frame


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
  def goto_goal_pose(self):

    # Construct goal msg
    goal_msg = self.make_goal_msg(self.goal_pose)

    cc_sleep_time = 5.0

    # Clear and come with me
    self.display_message('ΠΑΡΑΚΑΛΩ ΑΠΟΜΑΚΡΥΝΘΕΙΤΕ')
    call(['cvlc', '--no-repeat', '--play-and-exit', self.navigation_warnfile])
    rospy.loginfo('Waiting for %f sec before I clear costmaps', cc_sleep_time)
    rospy.sleep(cc_sleep_time)
    rospy.wait_for_service('/move_base/clear_costmaps')

    try:
      cc_srv = rospy.ServiceProxy('/move_base/clear_costmaps', srv_empty)
      cc_srv(srv_empty_req())
    except rospy.ServiceException as e:
      rospy.logerr('[%s] Service call failed: %s', self.pkg_name, e)


    rospy.loginfo('Waiting for %f sec after I clear costmaps', cc_sleep_time)
    rospy.sleep(cc_sleep_time)
    self.display_message('ΠΑΡΑΚΑΛΩ ΕΛΑΤΕ ΜΑΖΙ ΜΟΥ')

    rospy.loginfo('[%s] Sending goal and waiting for result', self.pkg_name)
    self.action_client.send_goal(goal_msg)

    # Play music while waiting for goal completion
    p = Popen(['cvlc', '--repeat', self.navigation_audiofile])

    # Wait for action completion
    self.action_client.wait_for_result()

    # Terminate audio file once on target
    p.terminate()


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

    # init transcript
    self.transcript_ = ''

    # The main screen's buttons and their text
    self.q_button_vec = []
    self.q_button_txt = []
    self.a_button_vec = []
    self.a_button_txt = []

    # Which exhibits have been viewed, in order.
    self.exhibit_codes_played = []

    # Get a move_base action client
    self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('[%s] Waiting to connect to move_base...', self.pkg_name)
    #self.action_client.wait_for_server()
    rospy.loginfo('[%s] Connected to move_base.', self.pkg_name)


  ##############################################################################
  def kill_root(self):
    rospy.logerr('[%s] Dialogue over', self.pkg_name)
    call(['bash', '/home/cultureid_user0/game_desktop_launchers/kill_all.sh'])
    self.root.destroy()
    os._exit(os.EX_OK)


  ##############################################################################
  def let_the_games_begin(self, q):

    rospy.loginfo('[%s] Starting dialogue over exhibit %s', self.pkg_name, self.exhibit_codes[q])

    # Load params for this specific exhibit (`exhibit_` + self.exhibit_codes[q])
    rospy.loginfo('[%s] Loading params', self.pkg_name)
    self.load_exhibit_params(self.exhibit_codes[q])
    rospy.loginfo('[%s] Done loading params', self.pkg_name)


    # Which exhibit is this?
    self.exhibit_codes_played.append(self.exhibit_codes[q])

    # The museum's map is huge and not everytime does the initial pose get
    # loaded. Set it here.
    if len(self.exhibit_codes_played) == 1:

      # Read start pose ...
      current_pose = rospy.get_param('~start_pose', '')

      # ... transform it into a message ...
      pose_msg = self.make_pose_msg(current_pose)

      # ... and publish it to /initialpose
      self.amcl_init_pose_pub.publish(pose_msg)

      rospy.sleep(1.0)

      # Do various motions to align the pose estimate to the actual pose
      self.move_to_calibrate_initialpose(2)


    # The parameters have been set, but we are at the end of an exhibit dialogue
    if len(self.exhibit_codes_played) > 1 and \
        self.exhibit_codes_played[-1] != self.exhibit_codes_played[-2]:
      self.goal_pose = rospy.get_param('~start_pose', '')

      if self.goal_pose == '':
        rospy.logerr('[%s] goal_pose for exhibit %s not set; aborting', \
            self.pkg_name, self.exhibit_codes[q])
        self.kill_root()
      else:
        self.goto_goal_pose()

    # Play this specific exhibit. This function blocks.
    #oe = one_exhibit.OneExhibit()

    rospy.logerr('[%s] Dialogue over exhibit %s over', self.pkg_name, self.exhibit_codes[q])

    # This game is over. Would you care to get to know another one?
    #self.main_screen()


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
  def main_screen(self):

    # new canvas
    canvas = self.new_canvas()
    self.set_canvas(canvas)

    # to frame panw sto opoio 8a einai ta koumpia
    frame = self.new_frame()
    self.set_frame(frame)
    frame = self.exit_button(frame)

    # All the choices for this question
    #choices = self.exhibit_titles


    # Show Q -------------------------------------------------------------------
    QButton = Tkinter.Button(frame,text='???',fg='white',bg='#E0B548',activeforeground='white',activebackground='#E0B548')
    self.q_button_vec.append(QButton)

    # The text of the question
    self.q_button_txt.append(self.transcript_)

    xNum = len(self.q_button_vec)
    yNum = 1


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

        self.q_button_vec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
        self.q_button_vec[counter].config(text=self.q_button_txt[counter])
        self.q_button_vec[counter].update()

        thisWidth = self.q_button_vec[counter].winfo_width()
        thisHeight = self.q_button_vec[counter].winfo_height()

        self.q_button_vec[counter].config(font=("Helvetica", 16))
        self.q_button_vec[counter].update()

        counter = counter+1




    # Show A -------------------------------------------------------------------
    self.a_button_txt.append(self.transcript_)
    this_butt = Tkinter.Button(frame,text='???',fg='#E0B548',bg='#343A40',activeforeground='#E0B548',activebackground='#343A40')
    self.a_button_vec.append(this_butt)

    xNum,yNum = self.get_x_y_dims(len(self.a_button_vec))

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

        if counter < len(self.a_button_vec):
          thisX = xG/2+xx*xWithGuard
          thisY = yG/2+yy*yWithGuard+1-yEff

          self.a_button_vec[counter].place(relx=thisX,rely=thisY,relheight=yB,relwidth=xB)
          self.a_button_vec[counter].config(text=self.a_button_txt[counter])
          self.a_button_vec[counter].update()

          thisWidth = self.a_button_vec[counter].winfo_width()
          thisHeight = self.a_button_vec[counter].winfo_height()
          self.a_button_vec[counter].config(font=("Helvetica", 24))
          self.a_button_vec[counter].update()

        counter = counter+1


  ##############################################################################
  def make_goal_msg(self, target_pose):

    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = target_pose[0]
    goal.target_pose.pose.position.y = target_pose[1]
    goal.target_pose.pose.position.z = target_pose[2]
    goal.target_pose.pose.orientation.x = target_pose[3]
    goal.target_pose.pose.orientation.y = target_pose[4]
    goal.target_pose.pose.orientation.z = target_pose[5]
    goal.target_pose.pose.orientation.w = target_pose[6]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal


  ##############################################################################
  def make_pose_msg(self, in_pose):

    out_pose = PoseWithCovarianceStamped()
    out_pose.pose.pose.position.x = in_pose[0]
    out_pose.pose.pose.position.y = in_pose[1]
    out_pose.pose.pose.position.z = in_pose[2]
    out_pose.pose.pose.orientation.x = in_pose[3]
    out_pose.pose.pose.orientation.y = in_pose[4]
    out_pose.pose.pose.orientation.z = in_pose[5]
    out_pose.pose.pose.orientation.w = in_pose[6]
    out_pose.header.frame_id = 'map'
    out_pose.header.stamp = rospy.Time.now()

    return out_pose


  ##############################################################################
  def move_to_calibrate_initialpose(self, num_tries):

    lf = Vector3(+0.1,0,0)
    lb = Vector3(-0.1,0,0)
    a = Vector3(0,0,0)
    twist_msg_bf_p = Twist(lf,a)
    twist_msg_bf_n = Twist(lb,a)

    l = Vector3(0,0,0)
    ap = Vector3(0,0,+1)
    an = Vector3(0,0,-1)
    twist_msg_ss_p = Twist(l,ap)
    twist_msg_ss_n = Twist(l,an)

    trn_nm = 8
    rot_nm = 4

    for i in range(num_tries):

      # Forth ------------------------------------------------------------------
      for i in range(trn_nm):
        self.raw_velocity_commands_pub.publish(twist_msg_bf_p)
        rospy.sleep(0.5)

      # Counterclockwise -------------------------------------------------------
      for i in range(rot_nm):
        self.raw_velocity_commands_pub.publish(twist_msg_ss_n)
        rospy.sleep(0.5)

      # Clockwise --------------------------------------------------------------
      for i in range(2*rot_nm):
        self.raw_velocity_commands_pub.publish(twist_msg_ss_p)
        rospy.sleep(0.5)

      # Counterclockwise -------------------------------------------------------
      for i in range(rot_nm):
        self.raw_velocity_commands_pub.publish(twist_msg_ss_n)
        rospy.sleep(0.5)

      # Back -------------------------------------------------------------------
      for i in range(trn_nm):
        self.raw_velocity_commands_pub.publish(twist_msg_bf_n)
        rospy.sleep(0.5)


  ##############################################################################
  def new_canvas(self):
    canvas = Tkinter.Canvas(self.root)
    canvas.configure(bg='white')
    #canvas.pack(fill=Tkinter.BOTH,expand=True)
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
  def read_transcript_file(self, event=None):

    with open(self.transcript_file_path,'r') as f:
      lines  = f.readlines()
      f.close()

    self.transcript_ = ''.join(lines)

    robot_talking = self.transcript_.find('[ROBOT]')
    human_talking = self.transcript_.find('[HUMAN]')

    # Both should be -1 if they are equal. There is no text to process
    if robot_talking == human_talking:
      return

    fg_color = ''
    if robot_talking != -1:
      self.transcript_ = self.transcript_[len('[ROBOT]')+1:]
      fg_color = '#E0B548'
    if human_talking != -1:
      self.transcript_ = self.transcript_[len('[HUMAN]')+1:]
      fg_color = '#FFFFFF'

    self.a_button_txt[0] = self.transcript_
    self.a_button_vec[0].config(text=self.a_button_txt[0])
    self.a_button_vec[0].config(fg=fg_color)

    if robot_talking != -1:
      self.a_button_vec[0].config(font=("Helvetica", 24))
    if human_talking != -1:
      self.a_button_vec[0].config(font=("Helvetica", 24, "italic"))

    # Wrap transcript text
    self.a_button_vec[0].config(\
        wraplength=self.a_button_vec[0].winfo_width()-10,justify="center")

    # Set new configuration
    self.a_button_vec[0].update()


  ##############################################################################
  def reset_file(self, file_str):
    with open(file_str,'w') as f:
      f.close()


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

  rospy.init_node('amth_exhibit_dialogue_node')

  try:
    ed = ExhibitDialogue()
    rospy.spin()

  except rospy.ROSInterruptException:
    print("SHUTTING DOWN")
    pass
