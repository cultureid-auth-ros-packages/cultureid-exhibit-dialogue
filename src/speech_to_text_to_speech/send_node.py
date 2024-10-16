#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

import requests
import json
import time
from speech_algorithm_for_rasa import *
from play_sound import *

class S2S():

  ##############################################################################
  # constructor
  ##############################################################################
  def __init__(self, transcript_file, robot_speech_time_file, lang, port):

    # Transcript of robot speech
    self.robot_speech_text_ = ''

    # Finalised transcript of human speech
    self.human_speech_text_ = ''

    # Write each transcript into a single file
    self.transcript_file_ = transcript_file

    # Write the duration of each robot speech into a file
    self.robot_speech_time_file_ = robot_speech_time_file

    # rasa language
    self.rasa_lang = lang
    self.el = 'el-GR'
    self.en = 'en-GB'

    # rasa port
    self.rasa_port = port

    # This is the meat
    self.talk_to_bot_via_rest()



  ##############################################################################
  def funcc(self, msg):

    headers = {'Content-type': 'application/json',}
    dct = {"sender": "tester", "message": msg}
    data = json.dumps(dct, indent=True)
    url = 'http://localhost:' + self.rasa_port + '/webhooks/rest/webhook'
    response = requests.post(url, headers=headers, data=data)

    if (response.status_code == 200) and (response.headers["content-type"].strip().startswith("application/json")):

      if self.rasa_lang == self.el:
        my_json = response.content.decode('utf8').replace("'", '"')
      if self.rasa_lang == self.en:
        my_json = response.content.decode('utf8')

      # Load the JSON to a Python list & dump it back out as formatted JSON
      data = json.loads(my_json)
      for bot_message in data:
        recipient_id = bot_message["recipient_id"]
        text = bot_message["text"]
        self.robot_speech_text_ = text

        # Write transcript of voice to transcript file
        self.write_file('[ROBOT] ' + self.robot_speech_text_, self.transcript_file_)

        # Speak text; blocks until all words are spoken
        text_to_speech(self.robot_speech_text_, self.robot_speech_time_file_, self.rasa_lang)

        # Signify end of speech
        self.reset_file(self.transcript_file_)
        self.write_file('[ROBOT] ' + self.robot_speech_text_ + ' [/ROBOT]', self.transcript_file_)

    return response


  ##############################################################################
  def talk_to_bot_via_rest(self):

    #------------------------- Robot has the initiative ------------------------
    #------------------- and speaks first because human is shy -----------------
    # This is a one-time only deal ---------------------------------------------
    while True:
      try:
        if self.rasa_lang == self.el:
          response = self.funcc("γειά")
        if self.rasa_lang == self.en:
          response = self.funcc("hi")

        if (response.status_code == 200) and (response.headers["content-type"].strip().startswith("application/json")):
          break

      except:
        print("[cultureid-exhibit-dialogue; s2s] First contact failed")
      time.sleep(1.0)



    #------------------------------ Come on Barbie -----------------------------
    while True:

      try:
        message, flag = speech_to_text(self.transcript_file_, self.rasa_lang)
        self.human_speech_text_ = message
        reset_file(self.transcript_file_)

      except:
        print("[cultureid-exhibit-dialogue; s2s] exception thrown while listening to human; pathetic")
        continue

      #if self.human_speech_text_ == "έξοδος":
        #break


      if flag == True:
        response = self.funcc(self.human_speech_text_)
      else:
        self.robot_speech_text_ = message

        # Write transcript of voice to transcript file
        self.write_file('[ROBOT] ' + self.robot_speech_text_, self.transcript_file_)

        # Speak text; blocks until all words are spoken
        text_to_speech(self.robot_speech_text_, self.robot_speech_time_file_, self.rasa_lang)

        # Signify end of speech
        self.reset_file(self.transcript_file_)
        self.write_file('[ROBOT] ' + self.robot_speech_text_ + ' [/ROBOT]', self.transcript_file_)



  ##############################################################################
  def write_file(self, content, file_str):
    with open(file_str,'w') as f:
      f.write(content)
      f.close()

  ##############################################################################
  def reset_file(self, file_str):
    with open(file_str,'w') as f:
      f.close()



################################################################################
# main
################################################################################
if __name__ == '__main__':

  transcript_file = '/home/cultureid_user0/catkin_ws/src/cultureid-exhibit-dialogue/transcripts/transcript.txt'
  speech_time_file = '/home/cultureid_user0/catkin_ws/src/cultureid-exhibit-dialogue/transcripts/speech_time.txt'

  rasa_lang = sys.argv[1]
  rasa_port = sys.argv[2]

  s = S2S(transcript_file, speech_time_file, rasa_lang, rasa_port)
