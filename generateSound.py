import os
from gtts import gTTS
from pydub import AudioSegment
import time

# Fonction de synthèse vocale pour convertir un texte en fichier audio
def text_to_speech(text):
 # Définition du nom du fichier audio de sortie
    output_file = "message_aveugle.mp3"

    # Création du fichier audio à partir du texte fourni
    tts = gTTS(text=text, lang='fr', slow=False)
    tts.save(output_file)

    # Chargement du fichier audio dans un objet AudioSegment
    audio = AudioSegment.from_file(output_file, format="mp3")

    # Conversion du fichier audio au format WAV avec un taux d'échantillonnage de 44100 Hz et 2 canaux
    audio.export(output_file, format="wav", parameters=["-ac", "2", "-ar", "44100"])

    # Lecture du fichier audio à l'aide de la commande "aplay"
    os.system(f"aplay -D plughw:2,0 {output_file}")
    time.sleep(1)

#test
