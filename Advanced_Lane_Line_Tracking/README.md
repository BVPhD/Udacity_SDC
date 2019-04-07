## Advanced Lane Finding
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, the goal is to write a software pipeline to identify the lane boundaries in a video. This done by completing the following tasks:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Apply a perspective transform to "birds-eye view" of lane.
* Use color transforms, gradients, to create a thresholded binary image of the lane edges.
* Detect left vs right lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

The final output is application of these steps to a video stream which is found in (project_video_annotated_smoothed.mp4). The details of how this was implemented is found in the jupter notebook in this directory or if you prefer a pdf version is also provided.
