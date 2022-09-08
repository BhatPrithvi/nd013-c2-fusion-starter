# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
Step 1: In the first step an extended Kalman filter was implemented to track a single target with only Lidar data over time.
This included making some settings in the file Loop_over_dataset.py. The major part was implementing the prediction and update steps.
On runnning the file loop_over_dataset.py after implementing the expected RMSE<0.35 was achieved. The result is stored in the folder Result_Images.

Step 2: In the second step, code was implemented in the file trackmanagement.py to initialize and delete tracks, set track state and tarck score. This step was the most challenging among all the steps as the matrix implementation had to consider 3D geometry particularly initialization of the variables track.x and track.P. There was need to backtrack through the courses and read the knowledge base. However, as a result a clear undertaanding of all the variable, functionality and dependent parameters was achieved. Some setting were also changed in loop_over_dataset.py.
On runnning the file loop_over_dataset.py after implementing the expected RMSE~0.8 was achieved. The result is stored in the folder Result_Images.

Step 3: Here a single nearest neighbor data association had to be implemented to associate measurements to tracks. The interesting part was the presence of more than one target. Some minor setting was done loop_over_dataset.py.This step facilitated understanding about the Mahalanobis distance and the gating function, which were used to calculate the association matrix and update the track state and track score. This step was relatively easy as the course covered all necessary steps and was very thorough. The resultant files after running loop_over_dataset.py meet the required specifications and can be seen in folder Result_Images.

Step 4: A non linear camera measurement model was implemented. It mainly included writing code to check if the input state vector x was in the field of vision. The second part was to implement nonlinear camera measurement fucntion h.There was some effort involved to transform position estimate from vehicle to camera coordinates and then project them to image coordinates. The part involved removing restriction to include the camera in the function generate_measurement and initializing camera measurement objects z and R.
There was some error in recording a movie. There is a line
frame = cv2.imread(os.path.join(path, images[0])) in the function make_movies in misc/Evaluations.
The height of images[0] is 480 while the height of the rest of the images is 430.
Changing this line to frame = cv2.imread(os.path.join(path, images[1])) solved the problem and the resutant video is saved in results and Result_Images.
The mean RMSE for two tracks< 0.25 and the image is seen in Result_Images.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
In theory there are concrete benfits to camera-lidar fusion compared to lidar only tracking. Using camera only has shortcomings like error in depth measurement, darkness or weather conditions like fog and rain. However it holds an upperhand in image classification and perceiving "D structures. Lidar on the other hand has very good performance in bad weather and darkness and gives minimak error in depth measurement. Combining both of them eliminates the mentioned disadvantages. The result can be seen here, where there is remarked decrease in mean RMSE error in atleast two tracks

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
In real life scenerios, even sensor fusion cannot absolutely eliminate all challenges. Allowances have to be made to consider that all sensors do not have the same visibility. Occlusions can also pose a problem. This was also seen in the tracks where Lidar had greater visibility as opposed to the camera, however mean RMSE was higher when only the LIDAR was used.

### 4. Can you think of ways to improve your tracking results in the future?
The tracking results can be improved by using data from more sensors as provided in the dataset. Global Nearest Neighbor (GNN) or Joint Probabilistic Data Association (JPDA) could be used instead of single nearest neighbor to calculate data associatin matrix. Parameters could be finetuned to decrease mean RMSE error.
