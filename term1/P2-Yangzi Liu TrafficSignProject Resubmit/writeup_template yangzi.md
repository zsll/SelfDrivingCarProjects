**Traffic Sign Recognition** 


---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./num_of_images.png "Visualization"
[image2]: ./enhanced.png "stop_dark"
[image3]: ./downloaded-signs2/1.jpeg "Traffic Sign 1"
[image4]: ./downloaded-signs2/2.jpeg "Traffic Sign 2"
[image5]: ./downloaded-signs2/3.jpeg "Traffic Sign 3"
[image6]: ./downloaded-signs2/4.jpeg "Traffic Sign 4"
[image7]: ./downloaded-signs2/5.jpeg "Traffic Sign 5"
[image8]: ./downloaded-signs2/6.jpeg "Traffic Sign 6"
[image9]: ./downloaded-signs2/7.jpeg "Traffic Sign 7"
[image10]: ./downloaded_result.png "downloaded_result"

## Rubric Points

---

###Data Set Summary & Exploration

####1. Basic summary of the data set.

I used the numpy library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

####2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how the number of images in each sign category of the training data set, test data set and valid data set. The range of number in each bin is between 200 - 2000. From the histogram we can easily tell most data are used for training, about 3 times of that for testing. And testing data number is usually 3 times of the validation data.

![alt text][image1]

###Design and Test a Model Architecture

####1. Preprocessing

I used classic image histogram equalization technique to process each of the image because many of the images are of low contrast or low brightness. Also I noticed most of the signs have a margin around 5px for each side. So I crop each image from 32x32 to 22x22 size. This will not only remove redundant parts that won't contribute for the training accuracy, but accelerate the training.

Here is an example of a traffic sign image before and after histogram equalization.

![alt text][image2]

We can tell before the enhancement many images like Stop sign are almost impossible to recognize. After histogram equalization it becomes much more bright and clear.


####2. Model architecture

My model mimic the LeNet model with slight modification to the input, stride and kernal dimensions.

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 22x22x3 RGB image   							| 
| Convolution 3x3     	| 1x1 stride, valid padding, outputs 20x20x8 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 10x10x8  				|
| Convolution 3x3	    | 1x1 stride, valid padding, outputs 8x8x20 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 4x4x20	  				|
| Flatten				| Input = 4x4x20, Output = 320					|
| Fully connected		| Input = 320. Output = 128      				|
| RELU					|												|
| Fully connected		| Input = 128. Output = 100      				|
| RELU					|												|
| Fully connected		| Input = 100. Output = 43      				|
| Softmax				| 		      									|
|						|												|
|						|												|
 


####3. Model Training

To train the model, I used the modified LeNet model stated as above. And train it against the reduced mean of cross entrophy, which is used to measure the overall predict quality. The training run 28 epochs, each with batch size of 100, and learning rate of 0.001. After epoch 15, the validatio accuracy stablized above 0.93 and eventaully hit 0.958.

####4. Approach discussion

My final model results were:
* training set accuracy of 0.996
* validation set accuracy of 0.958
* test set accuracy of 0.936

Modified LeNet was chosen as architechture. Since LeNet has been successfully applied to MNIST data set classification, which is similar to the traffic sign classification problem, it's very likely to work well for traffic sign classification. Training set accuracy is close to 1. This means the parameters trained can correctly recognize almost all signs in the training set. The validation accuracy grows gradually in each poch, which implies the SGD is moving toward the correct parameters. Eventually the test accuracy is also very high. Since test data is independent of training iterations, we can conclude the model is working well.

###Test a Model on New Images


Here are 7 German traffic signs that I found on the web:

![alt text][image3] ![alt text][image4] ![alt text][image5] ![alt text][image6] 
![alt text][image7] ![alt text][image8] ![alt text][image9]

Please note the downloaded images are not of standard size of 32x32. Before processing we need to scale them to fit into the model. This might affect the accuracy since the scale transform create extra hurdle. But in real life this might happen when looking at signs on your side.

####2. New Image Test Result

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Yield     			| Yield											| 
| Priority Road     	| Priority Road									| 
| 50 km/h limit	      	| 50 km/h limit					 				|
| Pedestrian			| Pedestrian	      							|
| General Warning		| General Warning	      						|
| Bumpy Road			| Bumpy Road	      							|
| Stop Sign      		| Stop sign   									| 

![alt text][image10]
The model was able to correctly guess 7 of the 7 traffic signs, which gives an accuracy of 100%. This is even better than the test accuracy. The reason behind this might be: the images in training/testing set are usually dark and blury, thus require enhancement. However, the downloaded images look much more sharp, thus is easier to classify.

####3. Predict Certainty

The code for making predictions on my final model is located in the 128th cell of the Ipython notebook.

For each image, the probability of the correct classification are all close to 1. As stated above, this is because usually images on web are of a better contrast and brightness.



