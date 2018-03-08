**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[undistort1]: ./output_images/undistort1.png "Undistorted1"
[undistort2]: ./output_images/undistort2.png "Undistorted2"
[undistort3]: ./output_images/undistort3.png "Undistorted3"
[undistort4]: ./output_images/undistort4.png "Undistorted4"

[warped1]: ./output_images/warped1.png "warped1"
[warped2]: ./output_images/warped2.png "warped2"
[warped3]: ./output_images/warped3.png "warped3"
[warped4]: ./output_images/warped4.png "warped4"

[apply_threshold1]: ./output_images/apply_threshold1.png "apply_threshold1"
[apply_threshold2]: ./output_images/apply_threshold2.png "apply_threshold2"
[apply_threshold3]: ./output_images/apply_threshold3.png "apply_threshold3"
[apply_threshold4]: ./output_images/apply_threshold4.png "apply_threshold4"

[histogram_sliding_window1]: ./output_images/histogram_sliding_window1.png "histogram_sliding_window1"
[histogram_sliding_window2]: ./output_images/histogram_sliding_window2.png "histogram_sliding_window2"
[histogram_sliding_window3]: ./output_images/histogram_sliding_window3.png "histogram_sliding_window3"
[histogram_sliding_window4]: ./output_images/histogram_sliding_window4.png "histogram_sliding_window4"

[detected_lanes1]: ./output_images/detected_lanes1.png "detected_lanes1"
[detected_lanes2]: ./output_images/detected_lanes2.png "detected_lanes2"
[detected_lanes3]: ./output_images/detected_lanes3.png "detected_lanes3"
[detected_lanes4]: ./output_images/detected_lanes4.png "detected_lanes4"

[projected_result1]: ./output_images/projected_result1.png "projected_result1"
[projected_result2]: ./output_images/projected_result2.png "projected_result2"


## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---


### Camera Calibration

#### 1. Calibration through locating corners in chessboards.

The code for this step is contained in the first code cell of the IPython notebook located in "./working\ copy/yangzi_pipeline.ipynb".

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][undistort1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

The coefficients are dumped to pickle file so that the calibration is not needed next time. Note here I picked four images with increasing lane detection difficulty. First one is just straight line. The other 3 are screenshots from the challenge.mp4, with dividing line, partially shadow and big trunk of shadow covered area. To demonstrate this step, The undistored result look like below:
![alt text][undistort2]
![alt text][undistort3]
![alt text][undistort4]

#### 2. Performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warper()`, which appears in lines 1 through 8 in the file `example.py` (output_images/examples/example.py) (or, for example, in the 3rd code cell of the IPython notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
    bottom_left = [200,720]
    bottom_right = [1130, 720]
    top_left = [570, 470]
    top_right = [722, 470]

    source = np.float32([bottom_left,bottom_right,top_right,top_left])
	
    bottom_left = [300,720]
    bottom_right = [940, 720]
    top_left = [300, 1]
    top_right = [940, 1]

    dst = np.float32([bottom_left,bottom_right,top_right,top_left])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 570, 470      | 300, 1        | 
| 200, 720      | 300, 720      |
| 1130, 720     | 940, 720      |
| 722, 470      | 940, 1        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.
![alt text][warped1]
![alt text][warped2]
![alt text][warped3]
![alt text][warped4]

#### 3. Create a thresholded binary image.  Provide an example of a binary image result.

Multiple channels are tested as below:
1. S Channel of HLS
2. Sobel x
3. Sobel y
4. Combined S channel and Sobel x
5. L Channel of LUV
6. B Channel of LAB
7. Combined L channel and B channel

Comparing these channels we can see Sobel y is very noisy and almost useless. S channel and Sobel x can together render meaningful results for the happy case, while in the challege.mp4 vedio, since the lane is divided into two different colors the x sobel channel has lots of noise as well. L Channel of the LUV space can ideally detect white lanes. B Channel of LAB does a good job finding yellow lines. So the final results is a union of B Channel, L Channel and S Channel. Out of the 3, L and B provide most of the info while S channel can only get a subset of B channel. So L and B channel could be sufficient for most of the scenarios. S is more like a safeguard. 

![alt text][apply_threshold1]
![alt text][apply_threshold2]

However, for the lanes under low light circumstances, like image 3 and 4, even this approach stopped working. I also tried common tricks like image histogram equalization. Seems it does not help at all.
![alt text][apply_threshold3]
![alt text][apply_threshold4]


#### 4. Sliding Window Search

Given the binary warped image, I calculated histogram to find the most likely lane location. And then performed sliding window search around this area as below:

![alt text][histogram_sliding_window1]
![alt text][histogram_sliding_window2]
![alt text][histogram_sliding_window3]
![alt text][histogram_sliding_window4]

After we found previous lane position, we just have to search around it. I used a margin of 100, as illustrated below:

![alt text][detected_lanes1]
![alt text][detected_lanes2]
![alt text][detected_lanes3]
![alt text][detected_lanes4]

#### 5. Calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

Once have polynomial fits and we can calculate the radius of curvature in method `measure_curvature`. Then called this method this in my code in `get_radius_curvature_and_center_offset` method. After getting the lane location, we just have to measure the distance bewteen the middle of the lane and center of the image to get the offset.

#### 6. Project back down onto the road such that the lane area is identified clearly.

I did inverse transform and project the lane area back to the original image. I also put radius of curvature of the lane and the position of the vehicle with respect to center to top of the image. I implemented this step in `hud` method.  Here is an example of my result on a test image:

![alt text][projected_result1]
![alt text][projected_result2]

---

### Pipeline (video)

#### 1. Compose pipeline
The final pipeline composes the steps as illustrated above.
1. Remove distortion
2. Perspective transform
3. Apply threshold of S, L and B channel
4. If we got fitting parameters, we use them directly to get x points to plot the curve. There are several cases we have to start a blind search:
    a. The very beginning
    b. The lane curvature does not look valid. As mentioned the valid range is between 500m and 5km. I give it a little bit more buffer.
    c. The lane curvature and vehicle position has a drastic change, which implies there is most likely a prediction issue.
5. Project result to original image

The pipeline provides precise and smooth prediction for the project_video.mp4 video (As shown in project_video_output.mp4).


---

### Discussion

The pipeline actually works well for project_video.mp4. But for result from the challenge_video.mp4 (challenge_video_output.mp4), we can tell there are some jumping frames where the predictions does not make too much sense. However, the lower part is mostly stable, since the lower part of the image are closer to the camera and thus more visible on the warped image. As expected, since the visual sensor won't help in dark light, prediction under the bridge gives ridiculous result. Averaging between adjacent frames might mitigate such issue. But considering this might be a very common senario during driving, ultimate solution should involve real time sensor fusion of multiple sources, or the CNN technique like Project 3. 