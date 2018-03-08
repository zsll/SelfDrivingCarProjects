import tensorflow as tf
from keras.layers import Dense, Flatten, Lambda, Activation, MaxPooling2D
from keras.layers.convolutional import Convolution2D
from keras.models import Sequential
from keras.optimizers import Adam
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import numpy as np
import cv2
import csv

tf.python.control_flow_ops = tf
    

# For each frame do the following transform:
# 1. Crop the top and bottom section of the image, since top part are usually environments not related to the navigation (Eg. trees, hills, posts) and bottom part is mostly occupied by the hood of the car.
# 2. Gaussion smoothing of kernel 3x3
# 3. Resize the result to a 64 by 64 square image.
# 4. Convert the image from RGB space to YUV space
def image_conversion(img):
    img = img[50:140,:,:]
    img = cv2.GaussianBlur(img, (3,3), 0)
    img = cv2.resize(img,(64, 64), interpolation = cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    return img

def data_generator(images, angles, batch_size):
    x,predict = ([],[])
    images, angles = shuffle(images, angles)
    while True:       
        for i in range(len(angles)):
            img = cv2.imread(images[i])
            angle = angles[i]
            img = image_conversion(img)
            x.append(img)
            predict.append(angle)
            if len(x) == batch_size:
                yield (np.array(x), np.array(predict))
                x, predict = ([],[])
                images, angles = shuffle(images, angles)
            # if has large steering angle, append the mirror as training data as well
            if abs(angle) > 0.3:
                x.append(cv2.flip(img, 1))
                predict.append(angle*-1)
                if len(x) == batch_size:
                    yield (np.array(x), np.array(predict))
                    x, predict = ([],[])
                    images, angles = shuffle(images, angles)

# model is Keras model
def addCNNAndMaxPoolingLayers(model, filter_size, kernel_size):
    model.add(Convolution2D(filter_size, kernel_size, kernel_size, border_mode='same', subsample=(2, 2)))
    model.add(Activation('relu'))
    # pool size and and strides are all the same
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(1, 1)))

# model is Keras model
def addFullyConnectedLayers(model, output):
    model.add(Dense(output))
    model.add(Activation('relu'))

### Make reading data ###

# loading csv file into memory
data_dir = '../P3_training_data/'

with open(data_dir + '/driving_log.csv', newline='') as f:
    csv_data = list(csv.reader(f, skipinitialspace=True,
                       delimiter=',', quoting=csv.QUOTE_NONE))
images = []
angles = []

#csv data: center,left,right,steering,throttle,brake,speed
for row in csv_data[1:]:
    center = row[0]
    left = row[1]
    right = row[2]
    angle = float(row[3])
    speed = float(row[6])
    if speed < 0.5:
        # if the car is almost still, training such data won't make too much sense 
        continue
        
    # For each row, generate left, center and right training data. Since the steering angle is based on the center camera,
    # Needs to add offset for the visual from left and right cameras
    images.append(data_dir + center)
    angles.append(angle)
    # Below +/-0.23 should be a constant generated from camera calibration. For now seems this assumed value works well
    images.append(data_dir + left)
    angles.append(angle + 0.23)
    images.append(data_dir + right)
    angles.append(angle - 0.23)

images = np.array(images)
angles = np.array(angles)
        
# split into train/test sets. Use 5% as testing data
images_train, images_test, angles_train, angles_test = train_test_split(images, angles, test_size=0.05, random_state=42)

print('Training size:', images_train.shape[0])
print('Testing size:', images_test.shape[0])

### Make the Keras model ###

# A mimic of https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
model = Sequential()
# Normalization
model.add(Lambda(lambda x: x / 127.5 - 1.0, input_shape=(64, 64, 3)))
# Convolutional and maxpooling layers
addCNNAndMaxPoolingLayers(model, 24, 5)
addCNNAndMaxPoolingLayers(model, 36, 5)
addCNNAndMaxPoolingLayers(model, 48, 5)
addCNNAndMaxPoolingLayers(model, 64, 3)
addCNNAndMaxPoolingLayers(model, 64, 3)

model.add(Flatten())

# Fully connected layers
addFullyConnectedLayers(model, 1164)
addFullyConnectedLayers(model, 100)
addFullyConnectedLayers(model, 50)
addFullyConnectedLayers(model, 10)
model.add(Dense(1))

model.compile(optimizer=Adam(0.001), loss="mse")

model.summary()

generator = data_generator(images_train, angles_train, 64)
validation_generator = data_generator(images_train, angles_train, 64)
history = model.fit_generator(generator, 
                              validation_data = validation_generator, 
                              nb_val_samples=6000, 
                              samples_per_epoch=24000, 
                              nb_epoch=10, 
                              verbose=1)                
print('Eventual Loss: ', model.evaluate_generator(data_generator(images_test, angles_test, 64), 64))

# Save model data
model.save('./model.h5')