from gtts import gTTS
import os

def text_to_speech(text):
    # Initialize gTTS with the text to convert
    speech = gTTS(text,lang='fr')
    # Save the audio file to a temporary file
    nameFile_speech = 'messageSpeech.mp3'
    speech.save(nameFile_speech)
    # Play the audio file
    os.system(nameFile_speech)


def speechListObstcale(list):

    for nameObject in list:
        text_to_speech(nameObject)
    