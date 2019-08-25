# Writeup

## Model Documentation

### How to select the lane

Usually, the vehicle tries to keep the current lane.
When there is an obstacle in front of the vehicle for a while, and either of the adjacent lanes is empty, the vehicle decides to do lane change.

### How to generate the path

After selecting the target lane, the vehicle looks at the front points in some distances.
Then, the path is generated connecting the points using Spline Interpolation.
To make the path smooth, the previous path is reused and the new path will be appended to the back.
