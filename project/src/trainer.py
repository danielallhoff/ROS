import numpy as np
import glob
import os
import sys
from keras import backend as K
from sklearn.model_selection import train_test_split
from keras.datasets import mnist
from keras.preprocessing.image import load_img
from keras.applications import VGG16
from keras.models import Input, Model
from keras.layers import Dense, Dropout, Activation, Flatten, MaxPooling2D, LeakyReLU, LSTM
from keras.layers import Reshape, Conv2DTranspose, concatenate, merge, ConvLSTM2D
from keras.layers.convolutional import Conv2D
from keras.callbacks import EarlyStopping
###########
#CONSTANTS#
###########
batch_size = 64
nb_classes = 4
epochs = 20
img_rows, img_cols, img_channel = 28, 28,1

size_sequence = 16
features = 300
###########
## CODE ##
###########

# 0 : '+'. 1 : 'l'. 2 : 'r' , 3 : 'm', 4 : 'DONOTHING'
dict_actions = {"+" : 0, "l" : 1, "r":2, "m":3, "n":4}

PATH_IMAGES = os.path.abspath('images')

'''
def get_input_from_name(filename):
    return filename.split('_')[3].split('.')[0]
'''

def load_data():
  y = []
  X = []

  with open("command_keys.txt") as fin:
    for action in file_in:
        number = dict_actions[action]
        one_hot_encoding = [0] * len(dict_actions)
        one_hot_encoding[number] = 1
        y.append(one_hot_encoding)


  # Load images
  for filename in glob.glob(PATH_IMAGES + "/*.jpg"):
      im = load_img(filename, target_size=[img_rows, img_cols, img_channels])
      X.append(np.asarray(im).astype('float32')/255.0)

  input_shape = (img_rows, img_cols, img_channel)

  max_data = max(len(y), len(X.size))

  X = X[:max_data]
  y = y[:max_data]

  return np.array(X), np.array(y), input_shape

def create_conv2d_model(input_shape=(img_rows,img_cols,img_channel), dropout = False):
    input = Input(shape = input_shape)
    conv1 = Conv2D(filters = 64, kernel_size = 5, strides=(2,2), padding='same', activation='relu', kernel_initializer='random_uniform')(input)
    if dropout:
        conv1 = Dropout(0.5)(conv1)
    conv2 = Conv2D(filters = 32, kernel_size = 5, strides=(2,2), padding='same', activation='relu', kernel_initializer='random_uniform')(conv1)
    
    if dropout:
        conv1 = Dropout(0.5)(conv1)
    data = Flatten()(conv2)
    

    output = Dense(32)(data)
    output = Dense(5)(output)
    
    model = Model(inputs= input, outputs=output)
    model.compile(optimizer='adam', loss ='categorical_crossentropy', metrics=['accuracy'])

    return model
#https://github.com/chinmayembedded/ConvLSTM2D/blob/master/conv2dlstm.ipynb
def create_conv2dlstm_model():
    input = Input(shape = input_shape)
    #input_shape = (batch_size, rows,cols, size_sequence)
    conv1 = ConvLSTM2D(filters = 64, input_shape=(None, img_rows,img_cols,size_sequence), kernel_size = (3,3), strides=(2,2), padding='same', kernel_initializer='random_uniform')(input)
    if dropout:
        conv1 = Dropout(0.5)(conv1)
    conv2 = ConvLSTM2D(filters = 32, kernel_size = (3,3), padding='same', kernel_initializer='random_uniform')(conv1)

    data = Flatten()(conv2)
    
    output = Dense(32)(data)
    output = Dense(5)(output)
    
    model = Model(inputs= input, outputs=output)
    model.compile(optimizer='adam', loss ='categorical_crossentropy', metrics=['accuracy'])

    return model


def train_model(model,X_train, y_train, batch_size = 32, epochs = 50, learning_rate = 0.001,validation_split=0.1):
    model.compile(optimizer = 'adam', loss='categorical_crossentropy', metrics=['accuracy'])
    es = EarlyStopping(monitor = 'val_loss', mode='min', verbose = 1)
    model.fit(X_train, y_train, batch_size = batch_size, epochs = epochs, callbacks=[es])
    return model

def load_pretrained_model():
    model_conv = VGG16(weights = 'imagenet', include_top = False, input_shape=(img_rows, img_cols,img_channel))
    model_conv.layers.pop()
    model = Model(inputs=model_conv.inputs, outputs = model_conv.layers[-1].output)
    flat1 = Flatten()(model.outputs)
    class1 = Dense(1024, activation='relu')(flat1)
    #Mover delante, izquierda, derecha, atrás y no mover
    output = Dense(5, activation='softmax')(class1)
    model = Model(inputs=model.inputs, outputs = output)
    model.summary()
    return model

def main(args):
    model = create_conv2d_model()

    X, y, input_shape = load_data()
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.25)
    
    #x_train = x_train.reshape(x_train.shape+(1,))
    #y_train = y_train.reshape(y_train.shape+(1,))
    
    print(x_train.shape)
    input_shape = (img_rows, img_cols,img_channel)
    
    model_trained = train_model(model, x_train, y_train)
    cmd_line = raw_input("Save? Y/N") 
    if(cmd_line == "Y" or cmd_line == "y"):
        model_trained.save("model_trained.h5")

if __name__ == '__main__':
    main(sys.argv)