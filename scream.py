# -*- coding: utf-8 -*-
"""
Created on Mon Oct 10 19:51:16 2022

@author: Indrajit
"""
import json
import os
import math
import librosa
import pickle
import sounddevice as sd
from scipy.io.wavfile import write
import librosa
import tensorflow as tf
import math
import matplotlib.pyplot as plt
import numpy
from sklearn import metrics

import json
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import precision_score, recall_score, f1_score, accuracy_score
import tensorflow.keras as keras

DATASET_PATH = "D:\\Capstone Resources\\emotiondata"
JSON_PATH = "D:\\Capstone Resources\\data_1.json"
SAMPLE_RATE = 22050
TRACK_DURATION = 5 # measured in seconds
SAMPLES_PER_TRACK = SAMPLE_RATE * TRACK_DURATION
def save_mfcc(dataset_path, json_path, num_mfcc=2, n_fft=2048, hop_length=512, num_segments=5):
    """Extracts MFCCs from music dataset and saves them into a json file along witgh genre labels.
        :param dataset_path (str): Path to dataset
        :param json_path (str): Path to json file used to save MFCCs
        :param num_mfcc (int): Number of coefficients to extract
        :param n_fft (int): Interval we consider to apply FFT. Measured in # of samples
        :param hop_length (int): Sliding window for FFT. Measured in # of samples
        :param: num_segments (int): Number of segments we want to divide sample tracks into
        :return:
        """

    # dictionary to store mapping, labels, and MFCCs
    data = {
        "mapping": [],
        "labels": [],
        "mfcc": []
    }

    samples_per_segment = int(SAMPLES_PER_TRACK / num_segments)
    num_mfcc_vectors_per_segment = math.ceil(samples_per_segment / hop_length)

    # loop through all genre sub-folder
    for i, (dirpath, dirnames, filenames) in enumerate(os.walk(dataset_path)):

        # ensure we're processing a genre sub-folder level
        if dirpath is not dataset_path:

            # save genre label (i.e., sub-folder name) in the mapping
            semantic_label = dirpath.split("/")[-1]
            data["mapping"].append(semantic_label)
            print("\nProcessing: {}".format(semantic_label))

            # process all audio files in genre sub-dir
            for f in filenames:

		# load audio file
                file_path = os.path.join(dirpath, f)
                signal, sample_rate = librosa.load(file_path, sr=SAMPLE_RATE)

                # process all segments of audio file
                for d in range(num_segments):

                    # calculate start and finish sample for current segment
                    start = samples_per_segment * d
                    finish = start + samples_per_segment

                    # extract mfcc
                    mfcc = librosa.feature.mfcc(signal[start:finish], sample_rate, n_mfcc=num_mfcc, n_fft=n_fft, hop_length=hop_length)
                    mfcc = mfcc.T

                    # store only mfcc feature with expected number of vectors
                    if len(mfcc) == num_mfcc_vectors_per_segment:
                        data["mfcc"].append(mfcc.tolist())
                        data["labels"].append(i-1)
                        print("{}, segment:{}".format(file_path, d+1))

    # save MFCCs to json file
    with open(json_path, "w") as fp:
        json.dump(data, fp, indent=4)
if __name__ == "__main__":
    save_mfcc(DATASET_PATH, JSON_PATH, num_segments=5)

import sounddevice as sd
from scipy.io.wavfile import write
import librosa
import tensorflow as tf
import math
import numpy
import numpy as np
import sys
import speech_recognition as ro
import json
import sounddevice as sd
from scipy.io.wavfile import write
import librosa

import math


import json
import numpy as np
from sklearn.model_selection import train_test_split
import tensorflow.keras as keras
from sklearn.metrics import accuracy_score
DATA_PATH = "D:\\Capstone Resources\\data_1.json"
def load_data(data_path):
    """Loads training dataset from json file.

        :param data_path (str): Path to json file containing data
        :return X (ndarray): Inputs
        :return y (ndarray): Targets
    """

    with open(data_path, "r") as fp:
        data = json.load(fp)

    # convert lists to numpy arrays
    X = np.array(data["mfcc"])
    y = np.array(data["labels"])

    print("Data succesfully loaded!")

    return  X, y

def predict(model, X):
    """Predict a single sample using the trained model
    :param model: Trained classifier
    :param X: Input data
    :param y (int): Target
    """
    new_model = tf.keras.models.load_model('my_model.h5')
    # add a dimension to input data for sample - model.predict() expects a 4d array in this case
    X = X[np.newaxis, ...] # array shape (1, 130, 13, 1)

    # perform prediction
    prediction = new_model.predict(X)

    # get index with max value
    predicted_index = np.argmax(prediction, axis=1)
    if predicted_index==0:
        label="no_scream"
    if predicted_index==1:
        label="scream"
   
    print("Predicted label: {}".format(label))
    return predicted_index

        
        
if __name__ == "__main__":
    DATA_PATH = "D:\\Capstone Resources\\data_1.json"
    
    X, y = load_data(DATA_PATH)

    # create train/test split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3)

    # build network topology
    model = keras.Sequential([

        # input layer
        keras.layers.Flatten(input_shape=(X.shape[1], X.shape[2])),

        # 1st dense layer
        keras.layers.Dense(512, activation='relu'),

        # 2nd dense layer
        keras.layers.Dense(256, activation='relu'),

        # 3rd dense layer
        keras.layers.Dense(64, activation='relu'),

        # output layer
        keras.layers.Dense(2, activation='softmax')
    ])

    # compile model
    optimiser = keras.optimizers.Adam(learning_rate=0.0001)
    model.compile(optimizer=optimiser,
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    model.summary()
    # print('Precision: %.3f' % precision_score(y_test, y_test_pred))
    # print('Recall: %.3f' % recall_score(y_test, y_test_pred))
    # print('Accuracy: %.3f' % accuracy_score(y_test, y_test_pred))
    

    # train model
    # model.fit(X_train, y_train, validation_data=(X_test, y_test), batch_size=32 , epochs=50)
    # #model.save('my_model.h5')

    # y_preds_test=model.predict(y_test)
    # print(accuracy_score(y_test,y_preds_test))



def takeCommand():
    
    r = ro.Recognizer()
     
    with ro.Microphone() as source:
        r.adjust_for_ambient_noise(source,duration=1)
        print("Listening...")
        r.pause_threshold = 1
        audio = r.listen(source)
  
    try:
        print("Recognizing...")   
        query = r.recognize_google(audio, language ='en-in')
        print(f"User said: {query}\n")
  
    except Exception as e:
        print(e)   
        print("Unable to Recognize your voice.") 
        return "None"
     
    return query
    
    
if __name__ == "__main__":

    
    

      # pick a sample to predict from the test set
    # y_test_pred=[]
    # for i in range(0,len(y_test)):
    #     X_to_predict = X_test[i]
    #     y_to_predict = y_test[i]
    #     y_test_pred.append(predict(model, X_to_predict,y_to_predict))
    # print("Accuracy of our model on test data : " , model.evaluate(X_test,y_test)[1]*100 , "%")

    # epochs = [i for i in range(50)]
    # fig , ax = plt.subplots(1,2)
    # train_acc = history.history['accuracy']
    # train_loss = history.history['loss']
    # test_acc = history.history['val_accuracy']
    # test_loss = history.history['val_loss']

    # fig.set_size_inches(20,6)
    # ax[0].plot(epochs , train_loss , label = 'Training Loss')
    # ax[0].plot(epochs , test_loss , label = 'Testing Loss')
    # ax[0].set_title('Training & Testing Loss')
    # ax[0].legend()
    # ax[0].set_xlabel("Epochs")
    # ax[1].plot(epochs , train_acc , label = 'Training Accuracy')
    # ax[1].plot(epochs , test_acc , label = 'Testing Accuracy')
    # ax[1].set_title('Training & Testing Accuracy')
    # ax[1].legend()
    # ax[1].set_xlabel("Epochs")
    # plt.show()
    #creating your own test data
    SAMPLE_RATE = 22050  # Sample rate
    DURATION = 5  # Duration of recording
    SAMPLES_PER_TRACK=SAMPLE_RATE * DURATION
    while True:
        query = takeCommand().lower()
        if 'exit' in query:
            sys.exit()
        print("speak")
        myrecording = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=2)
        sd.wait()  # Wait until recording is finished
        write('output.wav', SAMPLE_RATE, myrecording)  # Save as WAV file 
        from scipy.io.wavfile import read
        #a = read("output.wav")
        file = "output.wav"
        num_segments=5
        hop_length=512
        n_fft=2048
        n_mfcc=2
        num_samples_per_segment=int(SAMPLES_PER_TRACK/num_segments)
        num_mfcc_vectors_per_segment = math.ceil(num_samples_per_segment / hop_length)
        signal,sr=librosa.load(file,sr=SAMPLE_RATE)
        l=[]
        for s in range(num_segments):
            start_sample=num_samples_per_segment * s
            finish_sample=start_sample+num_samples_per_segment
            mfcc=librosa.feature.mfcc(signal[start_sample:finish_sample],sr=sr,n_fft=n_fft,n_mfcc=n_mfcc,hop_length=hop_length)
            mfcc=mfcc.T
        # if len(mfcc) == num_mfcc_vectors_per_segment:
        #     l.append(mfcc.tolist())
        #print(len(l))           
        l1=np.array(mfcc)
    
        #import numpy as n
        #l=np.array(a[1],dtype=float)
        #model = tf.keras.models.load_model('my_model.h5')
        # predict sample
        predict(model,l1)
        # import speech_recognition as r
        # rec = r.Recognizer()
        # with r.Microphone() as source:
        #     print('I\'M LISTENING...')
        #     audio = rec.listen(source, phrase_time_limit=5)
        #     try:
        #         text = rec.recognize_google(audio, language='en-US')
        #         print(text)
        #         if 'exit' in text:
        #             sys.exit()
        #     except:
        #         print('Sorry, I could not understand what you said.')
        
        

    
        # actual = numpy.random.binomial(1,.9,size = 1000)
        # predicted = numpy.random.binomial(1,.9,size = 1000)
    
        # confusion_matrix = metrics.confusion_matrix(y_test,y_test_pred)
    
        # cm_display = metrics.ConfusionMatrixDisplay(confusion_matrix = confusion_matrix, display_labels = [False, True])
    
        # cm_display.plot()
        # plt.show()
        











