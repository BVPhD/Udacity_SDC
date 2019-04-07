# **Finding Lane Lines on the Road** 

[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

The pipeline consists of the following steps:

1. separation into color channels
2. application of a Gaussian smoothing filter of width 9 pixels on each channel.
3. Canny edge detection on each channel with a min threshold of 100 and a max threshold of 130 to each channel.
4. Application of a polygon mask region of interest such that the front of the car is cut out, and is focusing on the lane directly in front of the camera for each channel.
5. Recombine each binary image into a single binary image.
6. Applying Hough line transformation with min line length of 7 pixels and a max line distance of 10 pixels and a resolution of 1 pixel radially and 1 radian.
7. Hough line points are separated into those which are associated with lines leaning from left to right and right to left.
8. The separated points are discarded if they are associated with a line with a slope magnitude below a threshold of $0.37$ in strength.
9. It's possible to have edges that lean left to right and not be associated with the left lane marker, so a heuristic additional filter is applied that when making the point separation that lines leaning from left to right must also be on the left side of the image, and vice versa for the right to left leaning lines.
10. After all filters are completed a best fit line is found for the points associated with the left lane and right lane.
11. The best fit line is extended from the bottom of the image to a chosen horizon point nearly half way up the image.
12. For each best fit line a residual error is computed between the line and the Hough line points associated with the best fit line.  If a large residual variance is found than the pipeline wasn't able to confidently find a single line and in the next step we factor in this confidence.
13. The end points of the best fit lines are incorporated into a moving average calculation such that the best fit lines which are drawn on the image are a weighted sum of the current frame computed end points and the previous frame end points where the weight is dependent on the best fit line confidence. This reduces single frame noise.
14. The lines are drawn on the image and the process repeats for the next frame.

A much more through explaintion is found in the juypter notebook.


![alt text][image1]


### 2. Identify potential shortcomings with your current pipeline

Major short comings with this method are:

1. Computation time, applying fits and computing variances and retaining states between frames leads to a lot of computation effort that may be too slow for practical application. I fear I may have over engineered this problem or fine tuned to the test data set.

2. The fit line is heavily dependent on edges found near the bottom the image, so any noise like rubber markings on concrete or any other noise has very strong influence on the projected line as that defect moves towards the bottom of the image.

3. Using the weighted after of multiple frames means the lane projection is lagged behind the actual frame rate.  This could be a problem if you expect there to be sharp turns in the lane markings.

4. It is not robust against missing edges, if an edge on one side of the mid line is not found numpy will throw an exception.

5. It will only work in sunny conditions and nearly perfect lane markings. This will not work in construction zones.

6. Im using global variables because I don't know how this f1 moviepy function is implemented.


### 3. Suggest possible improvements to your pipeline

1. I think using SVM lines would work better with the geometry of the problem rather than OLS fits of the data.

2. Rather than using the midline as a filter I should really be using a line which rotates with the turning angle of the vehicle.

3. The projection distance should be variable and not fixed at some point in the image, but I'm not sure how to implement this.

4. I need to employee some sort of normalization to each image so that there is constant contrast and brightness in each image, or some sort of standardization.

5. A MAP estimation of the lines where I incorporate a Bayesian prior of the fitted parameters would be more resistant to noise found in the images.
