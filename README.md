# liKalmanTracker
A toolbox using OpenCV 2.4.9 to do point multi-target Kalman tracking.

Author: Haozheng Li 
Email: sai-2008@qq.com or akaHaozhengLi@gmail.com

## 0 Why we need liKalmanTracker?
It's hard to do multi-target Kalman tracking in OpenCV because it only provides a single-target version.

All of us know the process of Kalman tracking, it needs a measurement state and a prediction state in each iteration.

The filter actually mix these two states into one state, then it considers the mix-state as the optimal estimate of targets' position.

Now the key problem is:

In each step/frame, the size/count of measurement state and prediction state are not equal.

For example, in (k-1)-th frame, we got N targets, which means we got N Kalman Filters. So we now have N prediction states.

Suppose in k-th frame, we have detected/measured M targets(M != N), then these M measurement states can not match N prediction states.

The result of that is the Kalman Filter iteration can not continue.

That's what liKalmanTracker deal with.

## 1 What does liKalmanTracker do?
As a result, liKalmanTracker deal with several problems in multi-target Kalman tracking as followed:

1) When new targets appear, M will increase, how to match them with N?

2) When targets leave, N will decrease, how to match them with M?


