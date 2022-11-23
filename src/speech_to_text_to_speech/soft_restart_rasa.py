#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

import requests
import json
import time
from speech_algorithm_for_rasa import *
from play_sound import *


##############################################################################
def funcc(msg):

  headers = {'Content-type': 'application/json',}
  dct = {"sender": "tester", "message": msg}
  data = json.dumps(dct, indent=True)
  response = requests.post('http://localhost:5005/webhooks/rest/webhook', headers=headers, data=data)

  if response_ok(response):

    my_json = response.content.decode('utf8').replace("'", '"')

    # Load the JSON to a Python list & dump it back out as formatted JSON
    data = json.loads(my_json)
    for bot_message in data:
      recipient_id = bot_message["recipient_id"]
      text = bot_message["text"]

      speech_time_file = '/home/cultureid_user0/catkin_ws/src/cultureid-exhibit-dialogue/transcripts/speech_time.txt'

      # Speak text; blocks until all words are spoken
      text_to_speech(text, speech_time_file)

  return response


##############################################################################
def response_ok(response):

  c1 = response.status_code == 200
  c2 = response.headers["content-type"].strip().startswith("application/json")

  if c1 and c2:
    return True
  else:
    return False


################################################################################
# main
################################################################################
if __name__ == '__main__':

  while True:
    try:
      print('[cultureid-exhibit-dialogue; s2s] rasa restart requested ...')
      response = funcc("επαναφορά σλοτ")

      if response_ok(response):
        print('[cultureid-exhibit-dialogue; s2s] ... rasa will restart')
        break

      time.sleep(0.2)
    except:
      print("[cultureid-exhibit-dialogue; s2s] ... but could not restart")
