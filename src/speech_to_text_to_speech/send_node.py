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
  def __init__(self, transcript_file):

    # Transcript of robot speech
    self.robot_speech_text_ = ''

    # Finalised transcript of human speech
    self.human_speech_text_ = ''

    # Write each transcript into a single file
    self.transcript_file_ = transcript_file

    # When starting-up reset the transcript file
    self.reset_file(self.transcript_file_)

    # This is the meat
    self.talk_to_bot_via_rest()


  ##############################################################################
  def funcc(self, msg):

    headers = {'Content-type': 'application/json',}
    dct = {"sender": "tester", "message": msg}
    data = json.dumps(dct, indent=True)
    response = requests.post('http://localhost:5005/webhooks/rest/webhook', headers=headers, data=data)

    if (response.status_code == 200) and (response.headers["content-type"].strip().startswith("application/json")):

      my_json = response.content.decode('utf8').replace("'", '"')

      # Load the JSON to a Python list & dump it back out as formatted JSON
      data = json.loads(my_json)
      for bot_message in data:
        recipient_id = bot_message["recipient_id"]
        text = bot_message["text"]
        self.robot_speech_text_ = text

        #print('ROBOT')
        #print(self.robot_speech_text_)
        self.write_file(self.robot_speech_text_, self.transcript_file_)

        # Speak text
        text_to_speech(self.robot_speech_text_)

    return response


  ##############################################################################
  def talk_to_bot_via_rest(self):

    #------------------------- Robot has the initiative ------------------------
    #------------------- and speaks first because human is shy -----------------

    while True:
      try:
        response = self.funcc("γειά")

        if (response.status_code == 200) and (response.headers["content-type"].strip().startswith("application/json")):
          break

      except:
        print("[cultureid-exhibit-dialogue; s2s] First contact failed")
      time.sleep(1.0)



    #------------------------------ Come on Barbie -----------------------------
    while True:

      try:
        message, flag = speech_to_text(self.transcript_file_)
        self.human_speech_text_ = message
        reset_file(self.transcript_file_)

        #print('HUMAN')
        #print(self.human_speech_text_)
      except:
        print("[cultureid-exhibit-dialogue; s2s] exception thrown while listening to human; pathetic")
        continue

      if self.human_speech_text_ == "έξοδος":
        break

      response = self.funcc(self.human_speech_text_)


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
  s = S2S(transcript_file)
