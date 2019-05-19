# import required packages
import pickle
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

import matplotlib.pyplot as plt
import cv2

from keras.models import Sequential, Model
from keras.layers import Lambda,Cropping2D,Input,Flatten,BatchNormalization,Activation
from keras.layers import Dense,Convolution2D,GlobalAveragePooling2D
from keras.models import load_model

import tensorflow as tf
import time

import numpy as np
from glob import glob
import os

np.random.seed(1337)

def uniform_file_paths(samples_path):    
    batch_samples = shuffle(samples_path)
   
    high_angle = 0.4
    # compute the angles based on the assumption that
    # the roadway is about 10 ft wide
    # and the center and left/right cameras are seperated by
    # 1 ft.
    # going from most left to most right along the lane
    camera_positions = np.array([5,4,3,1])
    angle_cuts = high_angle/5*(camera_positions[::-1])
    print("steering correction angles: ", angle_cuts*180/np.pi)
    
    for idx,batch_sample in enumerate(batch_samples):
        c_img = batch_sample[0]
        l_img = batch_sample[1]
        r_img = batch_sample[2]
        c_steering = float(batch_sample[3])
        directory = (batch_sample[0].split('/'))[-3]
       
        # apply aggressive steering to center correction
        if (directory == 'left'):
            # inside edge driving
            #correct steer right
            l_steering = c_steering+angle_cuts[3]
            r_steering = c_steering+angle_cuts[1]
            c_steering = c_steering+angle_cuts[2]
        elif (directory == 'right'):
            # outside edge driving
            #correct steer left
            l_steering = c_steering-angle_cuts[1]
            r_steering = c_steering-angle_cuts[3]
            c_steering = c_steering-angle_cuts[2]
        else:
            l_steering = c_steering+angle_cuts[1]
            r_steering = c_steering-angle_cuts[1]
                
        new_samples = np.array([c_img,l_img,r_img])
        new_angles = np.array([c_steering,l_steering,r_steering])
        if idx == 0:
            images = new_samples
            angles = new_angles
        else:
            images = np.concatenate((images,new_samples))
            angles = np.concatenate((angles,new_angles))

    images,angles = shuffle(images,angles)
    return images,angles

with open('./imgs_steer_paths.pkl','rb') as f:
    sample_paths = pickle.load(f)
    
X,y = uniform_file_paths(sample_paths)
 
print('spliting validation and training sets...')
ridx = np.random.choice(range(len(X)),int(np.floor(len(X)*0.8)),replace=False)
train_paths = X[ridx]
train_y = y[ridx]

cridx = [i for i in range(len(X)) if i not in ridx]

valid_paths = X[cridx]
valid_y = y[cridx]

def augment(X,y):
    X_flips = np.array([np.fliplr(xi) for xi in X])
    y_flips = np.array([-yi for yi in y])
    
    X_aug = np.concatenate((X,X_flips))
    y_aug = np.concatenate((y,y_flips))
    
    X_aug,y_aug = shuffle(X_aug,y_aug)
    
    return X_aug,y_aug

def data_gen_uniform_file(paths,targs,batch_size=32):
    num_rows = len(paths)
    while 1: # Loop forever so the generator never terminates
        paths,targs = shuffle(paths,targs)
        for offset in range(0, num_rows, batch_size):
            batch_paths = paths[offset:offset+batch_size]
            batch_targs = targs[offset:offset+batch_size]
            
            for idx,batch_sample in enumerate(batch_paths):
                c_img = plt.imread(batch_sample)
                c_img = cv2.cvtColor(c_img, cv2.COLOR_RGB2YUV)
                
                new_samples = np.array([c_img])
                if idx == 0:
                    images = new_samples
                else:
                    images = np.concatenate((images,new_samples))
            
            X_train_batch,y_train_batch = augment(images,batch_targs)
            yield X_train_batch, y_train_batch    


# Set our batch size
BATCH_SIZE = 128

# compile and train the model using the generator function
print('creating data generator...')
train_generator_obj = data_gen_uniform_file(train_paths,train_y, batch_size=BATCH_SIZE)
validation_generator_obj = data_gen_uniform_file(valid_paths,valid_y, batch_size=BATCH_SIZE)

def generate_nvidia_model():
    in_rows,in_cols = 160,320
    # define the CNN you want to use with preprocessing layers
    model = Sequential()
    #preprocessing
    model.add(Lambda(lambda x: (x-127.5)/127.5,input_shape=(in_rows,in_cols,3)))
    model.add(Cropping2D(cropping=((65,30),(0,0))))
    
    #NETWORK
    #conv_layer1
    model.add(Convolution2D(24,(5,5),strides=(2,2),padding='same'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    #conv_layer2
    model.add(Convolution2D(36,(5,5),strides=(2,2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    #conv_layer3
    model.add(Convolution2D(48,(5,5),strides=(2,2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    #conv_layer4
    model.add(Convolution2D(64,(3,3),strides=(1,1)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    #conv_layer5
    model.add(Convolution2D(64,(3,3),strides=(1,1)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))

    #FC layers
    # stack outputs in a flattened vector
    model.add(Flatten())

    model.add(Dense(100))
    model.add(Activation('relu'))

    model.add(Dense(50))
    model.add(Activation('relu'))

    model.add(Dense(10))
    model.add(Activation('relu'))
    #steering angle
    model.add(Dense(1))
     
    return model


print('generating model...')
my_model = generate_nvidia_model()
print('model generated.')
time.sleep(2)

print('compiling with loss=MSE, optimizer=ADAM ...')
my_model.compile(loss='mse', optimizer='adam')
print('constructing callbacks...')

train_mean = np.mean(train_y)
BASE_MSE = np.mean((train_y-train_mean)**2)

from keras.callbacks import EarlyStopping
stopper = EarlyStopping(monitor='val_loss', min_delta=0.001, patience=2)

EPOCHS = 5
time.sleep(2)
print('Ready to start training...')
print('EPOCHS: ',EPOCHS)
print('BATCH_SIZE: ',BATCH_SIZE)
print('BASE MSE: ', BASE_MSE)
print("GO...")
time.sleep(2)

history_object = my_model.fit_generator(train_generator_obj, 
                                     steps_per_epoch=np.ceil(len(train_paths)/BATCH_SIZE),
                                     validation_data=validation_generator_obj,
                                     validation_steps=np.ceil(len(valid_paths)/BATCH_SIZE), 
                                     callbacks=[stopper],
                                     epochs=EPOCHS,
                                     verbose=1)

model_path = './nvidia_model_fnl.h5'
print('Training done. Model saved to '+model_path)
my_model.save(model_path)

print('pickling training history data...')

train_loss = history_object.history['loss']
val_loss = history_object.history['val_loss']
training = np.array([train_loss,val_loss])
with open('./train_history.pkl','wb') as f:
    pickle.dump(training,f)

#model.save_weights('./my_model_weights.h5')

# save as JSON
#json_string = model.to_json()
#with open('./model.json','w') as f:
#    f.write(json_string)