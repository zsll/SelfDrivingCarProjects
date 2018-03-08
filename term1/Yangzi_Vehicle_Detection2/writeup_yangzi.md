**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./WriteUpImg/yangzi_feature_extraction.png
[image12]: ./WriteUpImg/yangzi_feature_extraction2.png
[image2]: ./WriteUpImg/yangzi_search_windows.png
[image3]: ./WriteUpImg/yangzi_multiple_windows.png
[image4]: ./WriteUpImg/yangzi_heatmap.png
[image5]: ./WriteUpImg/yangzi_heatmap1.png
[image6]: ./WriteUpImg/yangzi_heatmap2.png


## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points

###Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  


### Feature Extraction

####1. Selecting features and Tuning Parameters.

The code for this step is contained in 'extract_features' method. The feature contains HOG feature, color histogram, and binned color feature. As hinted in the lecture, I converted the  image into different color space including RGB (original), HSV, HSL, YCrCb and tried all channels and individual channels for each conversion. Using Hue of HSV perform well as expected since it can sort differentiate the vehicles from background. Using this feature alone can present an accuracy of 0.93. Using all channels of LUV can reach an accuracy of 0.989. Eventually all channels of YUV stands out with a training accuracy of 0.9924. A illustration of extracted LUV channels look like below.

![alt text][image1]

A illustration of extracted YUV channels look like below.

![alt text][image12]

Color histogram and bin features both help enhancing the accuracy. The combination I picked can eventaully hit an accuracy of 0.9924. However, more channels or bins will dramatically slow down the processing for each image. As the result the time to process one video might be extended from 1h to 3h. Training time will be longer with higher dimension of feature.

####2. Load Training Data and Persistate Extracted Features.

Read all data from the 'vehicles' and 'non-vehicles' folder provided. It takes more than 5m to extract features from 8k+ training images in each folder. So I read labeled data and store them in local pickle file.

####3. Train the classifier

Load the features stored in local pickle. The features are normalized and split into training and testing part. The training part of the features are feed into a SVM classifier. I used the LinearSVC classifier given in the lecture script. Seems the accuracy is sufficient for vehicle detection. The training speed is impressive. 80% of 17K is still over 13K. But the training only took 1.6s.

###Sliding Window Search

####1. Remove Image Distortion

Since we used the same video from previous project, we need to perform remove image distortion from each frame. The pickle file containing the camera matrix and distortion coefficients from previous calibration is reused here.

####2. Sliding Window Implementation

Here we define a sliding window function slide_window to generate a list of boxes with predefined parameters and a draw_boxes to draw the list of boxes on an image. I used 3 types of window with different sizes corresponding to the distance to the observing car. Note there are overlapping area since it's impossible to tell the size of the vehicles simply based on their location on the image. Larger 'xy_overlap' tends to give much better result since we are doing the window scan more frequently and is more likely to detect target and form the heat spot above threshold. However, this also requires more computing power and time.

![alt text][image2]

Note to save final video processing time, I removed the two windows on the left side. I tested it on these area and there are minimum false alarms for the given video.


####3. Run Classifier against all Windows

Each window defined by slide_windows function will be filtered by the trained predictor in search_windows function and classified as 'vehicle' or 'non-vehicle'. 

![alt text][image3]

####4. Filter False Positive through Heat Map
Note on the result above the windows are overlapping and many adjacent windows might all detect the same vehicle. We need to find a way to form a unified bounding box for each detected target. The heat map method is introduced to accumulate the count for the overlapping area in add_heap function. Only those above a given threshold will be considered valid prediction. I set the threshold to 2 in the final pipeline. This effectively eliminate majority of false alarms and effectively maintained real targets. 
Here's an example result showing the heatmap from a series of frames of video, the result of scipy.ndimage.measurements.label() and the bounding boxes then overlaid on the last frame of video:

![alt text][image4]
![alt text][image5]
![alt text][image6]

The final pipeline contains the image distortion, window search, heatmap thresholding and labeling. Although the heatmap looks valid in the previoius section, in individual frames many false positive appeared in the processed video. The correct tracking boxes also look flaky. A straight-forward way to improve is that using a deque to store history heat map boxes. Only boxes recent 10 frames would be stored. Only when accumulated heat values exceed a threashold will this position be considered a true detection. This method will greatly mitigate false positives and also renders much smoother tracking boxes.

### Video Implementation

####1. Here's a [link to my video result](https://youtu.be/7kuZ2GwMCNE)



---

###Discussion

From the video result we can see the both the white and black car are precisely captured in most of the frames. So we know the feature extraction is not biased by color. However, the shadow area tends to give false alarm which seems implying the LUV method is vunerable to lights changes. A combination of more color spaces and channels might be more robust while it would be more resource demanding and may further harm the processing speed. It is vital to strike a balance between them. Cars can be captrued when they are close or when getting further. This means the window size division based on distance is valid. I only give 3 groups windows with small overlapping area. More window with different sizes and larger overlapping area will be sure to present better result. In that case, heat map threshold might need to be adjusted higher since the same area will be examed much more times. Note the given heat map method can hardly provide bounding box with consistant size. It's caused by the nature of the heat map method. It's possible to further process the detected area to give more precise result. Another possibility is to apply the lane detecting technique from previous project and tell the distance of each area. Then we can better decide the search window size for a given location. This will potentially present more precise predict and save computing power. A typical use case is like: for the given video, obvious on left of the yellow line there would be no vehicles and there is no need to place windows there.

