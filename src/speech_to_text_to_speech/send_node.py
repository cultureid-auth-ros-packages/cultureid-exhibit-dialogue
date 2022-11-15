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
  def __init__(self):

    self.robot_speech_text_ = ''
    self.human_speech_text_ = ''
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
          print('ROBOT')
          print(self.robot_speech_text_)
          text_to_speech(self.robot_speech_text_)

    return response


  ################################################################################
  def talk_to_bot_via_rest(self):


    #------------------------- Robot has the initiative ------------------------
    #------------------- and speaks first because human is shy -----------------

    while True:
      try:
        response = self.funcc("γειά")

        if (response.status_code == 200) and (response.headers["content-type"].strip().startswith("application/json")):
          break

      except:
        print("failed")
      time.sleep(1)



    #------------------------------ Come on Barbie -----------------------------
    while True:

      try:
        message, flag = speech_to_text()
        self.human_speech_text_ = message

        print('HUMAN')
        print(self.human_speech_text_)
      except:
        print("error exception caught")
        continue

      if self.human_speech_text_ == "έξοδος":
        break

      response = self.funcc(self.human_speech_text_)




################################################################################
# main
################################################################################
if __name__ == '__main__':

  s = S2S()
