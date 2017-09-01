# Toolbox_liKalmanTracker
#### A toolbox using OpenCV 2.4.9 to do point multi-target Kalman tracking.

#### Author: Haozheng Li
#### Email: sai-2008@qq.com or akaHaozhengLi@gmail.com

## 0 Why we need liKalmanTracker?
It's hard to do multi-target Kalman tracking in OpenCV because it only provides a single-target version.

All of us know the process of Kalman tracking, it needs a measurement state and a prediction state in each iteration.

The filter actually mix these two states into one state, then it considers the mix-state as the optimal estimate of targets' position.

Now the key problem is:

#### In each step/frame, the size/count of measurement state and prediction state are not equal.

For example, in (k-1)-th frame, we got N targets, which means we got N Kalman Filters. So we now have N prediction states.

Suppose in k-th frame, we have detected/measured M targets(M != N), then these M measurement states can not match N prediction states.

The result of that is the Kalman Filter iteration can not continue.

That's why "liKalmanTracker" is here.
  
## 1 How to use liKalmanTracker?

The algorithm is coded by C++, OpenCV 2.4.9 in Visual Studio 2013.

I coded it as a Class called liKalmanTracker.

Just open the project and run main.cpp, a multi-target Kalman tracking demo will run.

All you need to do is as followed:

#### 1) Initialize a liKalmanTracker

API: liKalmanTracker(targetSize, targetName);

#### 2) Feed it with measurement sequence which is detected by your classifier or other algorithm

API: tracker.track(measurement);

#### 3) Print or show the result image, or directly get the tracking sequence

APIs: tracker.print(nFrameCount); tracker.show(dst_tracking, 0); tracker.trackment();

## 2 What does liKalmanTracker do?
As a result, liKalmanTracker deal with several problems in multi-target Kalman tracking as followed:

#### 1) When new targets appear, M will increase, how to match them with N? (M > N)

#### 2) When targets leave, M will decreased, how to match them with N? (M < N)

#### 3) Even if M == N, how to deal with false detected targets?

In one word, <liKalmanTracker> deal with false-rejected and false-accepted problems in multi-target Kalman tracking problem.
  
It will automatically match measurement states and prediction states, mix them and output the optimal estimate of targets' position.

## 2 How does liKalmanTracker work?

I build a confidence model for all targets.

The confidence/score means how important one target is. It determins the level of a target to be tracked.

For example, a new appearance target's confidence should be 0.

Another example, if one target can not be detected by the classifier or other algorithm, its confidence should be decreased.

As a result, I manage to build such a model to let liKalmanTracker decise which target should be tracked.

#### 1) M > N

It means new targets appear.

Match N Kalman Filters with N measurement based on mini L2 distance. Targets which successfully matched, confidence increase.

Extend and initialize (M - N) new Kalman Filters, confidence = 0.

Now M == N, cool!

#### 2) M < N

It means targets is leaving or false rejected.

Match M measurement with M Kalman Filters based on mini L2 distance. Targets which successfully matched, confidence increase.

Extend (N - M) measurement with N's prediction states, but confidence decrease.

Now M == N, cool!

#### 3) M == N

It means M match N, cool!

#### 4)/0) Opps! Wait! From 1) to 3), false accepted may occur!

So before 1) to 3), firstly we must deal with false accepted problem.

If some measurement are really far from every Kalman Filters, then just consider them as new Kalman Filters.

So now make 4) as 0). Finish it before step 1).

After all of these step, M measurement states can match N prediction states, and all the Kalman Filters will work well.

## 3 Does liKalmanFilter work well?

Sure!

If you want to know more details about the multi-target Kalman traking algorithm, or want to know more about my work,

#### please read the pdf paper "Video monitoring method of escalator entrance area based on Adaboost and codebook model" in this project.


# Have fun!! :)

