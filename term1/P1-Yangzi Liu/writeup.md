---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./test_images_output/solidWhiteCurve.jpg "Grayscale"

---

### 1. Reflection

My pipeline consisted of 5 steps:

a. Apply Gaussian smooth to original image.

b. Run Canny to detect edges out of result of step a.

c. Mask step b's outcome with a trapezoid area at the bottom.

d. Apply Hough transform to the masked section from step c.

e. From step d, what we get are a series of non-consecutive lines. Instead directly drawing all lines directly, I improved draw_line() function to present TWO SOLID LINEs based on these scattered lines. The two lines start from bottom and reach top of the trapezoid defined by the region of interest. The input lines are divided into two groups according to their slopes. The slope and offset of the two lines will be calculated through Least squares polynomial fit of the endpoints of the lines in the group. The eventual outcome are two solid lines as given in emample video.

![solidWhiteCurve][image1]


### 2. Identify potential shortcomings with your current pipeline


One potential problem of the overall approach is that all parameters are fixed values. But real world is so complicated which might requrie more adaptive parameters. For example, the final code does not work well for the challenge video, when passing through some area in shadow. Bad weather, low light, and bumppy surface definitely need different set of parameters.

Besides, in all videos, even including the challenging one, the roads are not crowded so there are seldom blocking issue from other cars. In case of heavy traffic jam, the cameras would have very limited sight on the road and Canny would generate lots of noise.

Another issue is that some roads might not have such clear solid dividers. They might be worn or dated. Or they might be just small dots instead of painted lines. The pipeline won't be very useful in such situation. 


### 3. Suggest possible improvements to your pipeline

A possible improvement would be to exclude some lines provided by Hough transform but are clears not part of the lane marks. For example: a bump on the road is horizontal and thus clearly not useful for lane detection. Other traffic marks like arrows or characters/words should be excluded as well.

Another potential improvement could be: Due to noise in the challenge video, the generated lines fly away in some frames. But we can actually assume the change of lines should be continuous, which means the delta of the offset and slope of the lines should fall into a valid range. If we can enforce this, we will mitigate such drastic change of lines and potentially improve the result.