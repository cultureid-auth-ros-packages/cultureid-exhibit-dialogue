Two things:

(a) the GUI

```
$ roslaunch cultureid_exhibit_dialogue avanti_dialogue.launch
```

(b) the speech-to-speech thing (assumes rasa is running)


```
$ roscd cultureid_exhibit_dialogue
$ cd src/speech_to_text_to_speech
$ python3 send_node.py
```
