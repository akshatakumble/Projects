"""Speech_Sentiment_Analyzer.ipynb
Original file is located at
https://colab.research.google.com/drive/10RAgRI4KxvYv-qd1AC1BbXwHyD5hi7DZ
"""

import speech_recognition as sr
import webbrowser

print("Music Recommendation Based on Emotion")

def recognize_speech_from_mic(recognizer, microphone):

    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source) # #  analyze the audio source for 1 second
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #   update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable/unresponsive"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response

recognizer = sr.Recognizer()

mic = sr.Microphone(device_index=1)

response = recognize_speech_from_mic(recognizer, mic)

print('\nSuccess : {}\nError   : {}\n\nText from Speech\n{}\n\n{}' \
          .format(response['success'],
                  response['error'],
                  '-'*17,
                  response['transcription']))

temp_str = ''
i=0
#records_all = []
print('How are you feeling?')
print('Sad, excited, angry?!')
while (i==0):
    print('Answer in a word!')
    response = recognize_speech_from_mic(recognizer, mic)
    if response['success']:
        temp_str = response['transcription']
        print('You said :' ,temp_str )
        if (i==0):
            if temp_str=='sad':
                webbrowser.open(f"https://www.youtube.com/results?search_query=happy+songs")
                #records_all.append(temp_str)
            if temp_str=='excited':   
                webbrowser.open(f"https://www.youtube.com/results?search_query=excited+songs")
                #records_all.append(temp_str)
            if temp_str=='angry':   
                webbrowser.open(f"https://www.youtube.com/results?search_query=calm+songs")
                #records_all.append(temp_str)
                
    i+=1

#print('Your emotions')
#for i in range(len(records_all)):
 #   print(records_all[i], end=" ")
'''
from textblob import TextBlob

def get_sentiment(sentx):
        
        #Utility function to classify sentiment of passed tweet
        #using textblob's sentiment method

        # create TextBlob object of passed text
        analysis = TextBlob(sentx)
        # set sentiment
        if analysis.sentiment.polarity > 0:
            return ('positive')
        elif analysis.sentiment.polarity == 0:
            return ('neutral')
        else:
            return ('negative')

# Select from collection
sentiments_total = {'neutral': 0 , 'positive' : 0 , 'negative':0}
for recd_sent in records_all:
    sentiment = get_sentiment(recd_sent)
    print (sentiment,'===>',recd_sent)
    sentiments_total[sentiment] = sentiments_total[sentiment] + 1
    print('###########################################')
print(sentiments_total)
'''

'''
from matplotlib import pyplot as plt
slices = [sentimets_total['neutral'],sentimets_total['positive'],sentimets_total['negative']]
activities = ['Neutral','Positive','Negative']
cols = ['c','m','r',]

plt.pie(slices,
        labels=activities,
        colors=cols,
        shadow= True,
        autopct='%1.1f%%')

plt.title('Sentiment Analysis')
plt.legend()
plt.show()
'''

