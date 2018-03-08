**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: before.png "before transform"
[image2]: after.png "after transform"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model is a mimic of the [nvidia's end-to-end learning](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf). I adopt this model since it has a very similar input (visual from 3 cameras) and output (steering angles). It aims to minimize the mean squared error between the steering command
output by the network and those in the training data. The model has 9 layers: a normalization layer, 5 convolutional layers and 3 fully connected layers (model.py lines 106-131). The CNN layers are followed by a max pooling layer of pool size of 2 and stride 1. The First 3 CNN layers are of kernel size 5, and the last 3 are of kernel size 3. The model includes RELU layers to introduce nonlinearity (code line 53, 60), and the data is normalized in the model using a Keras lambda layer (code line 108).


#### 2. Attempts to reduce overfitting in the model

Note that during the training, validation loss is slighting less than training loss, implying there is no overfitting, no Dropout was applied into the model.
```
____________________________________________________________________________________________________
Epoch 1/10
24000/24000 [==============================] - 109s - loss: 0.0227 - val_loss: 0.0141
Epoch 2/10
24000/24000 [==============================] - 93s - loss: 0.0112 - val_loss: 0.0106
Epoch 3/10
24000/24000 [==============================] - 83s - loss: 0.0092 - val_loss: 0.0084
Epoch 4/10
24000/24000 [==============================] - 85s - loss: 0.0089 - val_loss: 0.0076
Epoch 5/10
24000/24000 [==============================] - 80s - loss: 0.0088 - val_loss: 0.0082
Epoch 6/10
24000/24000 [==============================] - 81s - loss: 0.0080 - val_loss: 0.0067
Epoch 7/10
24000/24000 [==============================] - 83s - loss: 0.0083 - val_loss: 0.0072
Epoch 8/10
24000/24000 [==============================] - 76s - loss: 0.0075 - val_loss: 0.0064
Epoch 9/10
24000/24000 [==============================] - 77s - loss: 0.0068 - val_loss: 0.0065
Epoch 10/10
24000/24000 [==============================] - 77s - loss: 0.0067 - val_loss: 0.0058
```
The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track (run7.mp4).


#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 125).

#### 4. Appropriate training data

Training data was just the provided csv file and image frames. I tried to create new data as a supplement, which proves not too helpful due to my awful driving skills that I don't want to be cloned. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to use a well-established research result to train a model using image frames labeled with steering angles, hoping that when cloning the driving behavior, the model could predict proper driving direction to keep the car between the curbs.

My first step was to use a convolution neural network model similar to the [nvidia's end-to-end learning](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf). I thought this model might be appropriate because it also used the image frames from 3 front cameras to predict steering angles. And it presents persuasive end-to-end result. Note this model has no Dropout layer. From the training print out, the mean squared error on the training set and mean squared error on the validation set are both maintained at a low level. This implied that the model was not overfitted.

Then I have to process provided trianing images for feature extraction. To keep useful info (like the middle part of the frames, where we can see road surface), the top and bottom section of the image are truncated, since top part are usually environments not related to the navigation (Eg. trees, hills, posts) and bottom part is mostly occupied by the hood of the car. The images then goes through Gaussian blur and an resize to small square 64 by 64 dimension. Note that in the original paper, the input size is 200 X 66. Experiment shows smaller input size although erased some details, it won't impact the performance much. More importantly, smaller-sized input greatly improved training speed. The original 200 x 66 dimension will take over 200s for each epoch on my Macbook, while the 64 x 64 image reduced this to ~70s. In the end the images are converted to YUV space as the [nvidia'paper](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) did.

Next I have to tune the parameters including the input image size, CNN filter size, kernel size and max pooling size and strides. I tried to add Dropout layer as well. As mentioned above, since there was no obvious overfitting, this does not make a any performance improvement.

The final step was to run the simulator to see how well the car was driving around track one. Sometimes the car will swing back and forth on the track after the turn. This implies a lack of training data that recover from edge of the road back to driving straight in the center of the road. This is very obvious at turn points where curbs has the red and white reminding marks (in both run7.mp4 and run9.mp4). Somehow the car can adjust itself back to track to prevent driving onto the curb. If the throttle value is too large, the car is more likely to rush out of the track at these sharp turns. This also indicates the lack of training data with larger steering angles. To improve the driving behavior in these cases, I produced more data in training generator for sharper turns (model.py line 42) by mirroring the image. Also in drive.py, I used a constant throttle 0.25 (drive.py line 25). I tried to apply brake in case of high speed and large steering angle. However, this does not make lots of improvement.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 106-131) consisted of a convolution neural network with the 1 Normalize Layer 5 2D CNN layer and 4 fully connected layers. Note all layers between the input and output are connected with ReLu activation to form the neural network. Below is a summary of the Keras model.

```sh
____________________________________________________________________________________________________
Layer (type)                     Output Shape          Param #     Connected to                     
====================================================================================================
lambda_1 (Lambda)                (None, 64, 64, 3)     0           lambda_input_1[0][0]             
____________________________________________________________________________________________________
convolution2d_1 (Convolution2D)  (None, 32, 32, 24)    1824        lambda_1[0][0]                   
____________________________________________________________________________________________________
activation_1 (Activation)        (None, 32, 32, 24)    0           convolution2d_1[0][0]            
____________________________________________________________________________________________________
maxpooling2d_1 (MaxPooling2D)    (None, 31, 31, 24)    0           activation_1[0][0]               
____________________________________________________________________________________________________
convolution2d_2 (Convolution2D)  (None, 16, 16, 36)    21636       maxpooling2d_1[0][0]             
____________________________________________________________________________________________________
activation_2 (Activation)        (None, 16, 16, 36)    0           convolution2d_2[0][0]            
____________________________________________________________________________________________________
maxpooling2d_2 (MaxPooling2D)    (None, 15, 15, 36)    0           activation_2[0][0]               
____________________________________________________________________________________________________
convolution2d_3 (Convolution2D)  (None, 8, 8, 48)      43248       maxpooling2d_2[0][0]             
____________________________________________________________________________________________________
activation_3 (Activation)        (None, 8, 8, 48)      0           convolution2d_3[0][0]            
____________________________________________________________________________________________________
maxpooling2d_3 (MaxPooling2D)    (None, 7, 7, 48)      0           activation_3[0][0]               
____________________________________________________________________________________________________
convolution2d_4 (Convolution2D)  (None, 4, 4, 64)      27712       maxpooling2d_3[0][0]             
____________________________________________________________________________________________________
activation_4 (Activation)        (None, 4, 4, 64)      0           convolution2d_4[0][0]            
____________________________________________________________________________________________________
maxpooling2d_4 (MaxPooling2D)    (None, 3, 3, 64)      0           activation_4[0][0]               
____________________________________________________________________________________________________
convolution2d_5 (Convolution2D)  (None, 2, 2, 64)      36928       maxpooling2d_4[0][0]             
____________________________________________________________________________________________________
activation_5 (Activation)        (None, 2, 2, 64)      0           convolution2d_5[0][0]            
____________________________________________________________________________________________________
maxpooling2d_5 (MaxPooling2D)    (None, 1, 1, 64)      0           activation_5[0][0]               
____________________________________________________________________________________________________
flatten_1 (Flatten)              (None, 64)            0           maxpooling2d_5[0][0]             
____________________________________________________________________________________________________
dense_1 (Dense)                  (None, 1164)          75660       flatten_1[0][0]                  
____________________________________________________________________________________________________
activation_6 (Activation)        (None, 1164)          0           dense_1[0][0]                    
____________________________________________________________________________________________________
dense_2 (Dense)                  (None, 100)           116500      activation_6[0][0]               
____________________________________________________________________________________________________
activation_7 (Activation)        (None, 100)           0           dense_2[0][0]                    
____________________________________________________________________________________________________
dense_3 (Dense)                  (None, 50)            5050        activation_7[0][0]               
____________________________________________________________________________________________________
activation_8 (Activation)        (None, 50)            0           dense_3[0][0]                    
____________________________________________________________________________________________________
dense_4 (Dense)                  (None, 10)            510         activation_8[0][0]               
____________________________________________________________________________________________________
activation_9 (Activation)        (None, 10)            0           dense_4[0][0]                    
____________________________________________________________________________________________________
dense_5 (Dense)                  (None, 1)             11          activation_9[0][0]               
====================================================================================================
Total params: 329,079
Trainable params: 329,079
Non-trainable params: 0

```

#### 3. Creation of the Training Set & Training Process

I only used provided training data for it's hard to use the manual mode to generate smooth driving data. Unfortunately I am not a good racing game player, especially using the keyboard. 2nd track is even more challenging to drive on.

Even neural network does not emphasize on feature extraction, I still need to feed most valuable part of each frame to enhance both performance and training rate. Below is an example of provided camera shooting frame.

![Before][image1]

To generate valuable steering data on an empty open road, any object beyond the horizon would not make sense. Similar at the very bottom of the image: we have a fixed engine hood that does not provide much information. So I did a crop to only keep the part in the middle. After that, Gaussian blurring was applied to smooth the image details. Then we will reshape the image to 64 by 64 and eventually convert the image to YUV space. After this series of conversion, the result look like below.

![After][image2]

From the result we can tell the good thing would be the curbs are clearing marked. However the yellow lines are kind of dimmed. This should not matter too much for track one, where road condition is less complicated and we only have to keep the car in middle of road.

The processed data are split into training and testing data using train_test_split (model.py line 121). 5% of the data are used for testing and rest are for training and validation. The training and validation data are feed into a data generator (model.py line 27) after shuffling. Here since most training data are for driving on straight lanes, I produced more data in training generator for sharper turns (model.py line 42) by flipping the image.

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. I used 10 epochs, after which the loss no long drop dramatically. I used an adam optimizer so that manually training the learning rate wasn't necessary.
