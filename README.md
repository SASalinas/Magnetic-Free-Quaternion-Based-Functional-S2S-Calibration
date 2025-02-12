# Magnetic-Free-Quaternion-Based-Functional-S2S-Calibration
This repository contains the code and an example for performing Sensor-to-Segment Calibration using Magnetic-Free Quaternion-Based data.
![Method](https://github.com/user-attachments/assets/f835075f-3c46-4b32-a4f0-51268d4d0c5e)
Sensor-to-segment calibration methodology proposed to calculate the misalignment between one sensor and its respective segment ($^{Seg(i)}q_{S(i)}$), as well as the rotation between the global coordinate reference systems of two inertial sensors ($^{WS(i-1)}q_{WS(i)}$), where the $i$--sensor is attached to the $i$--segment, while ${(i-1)}$--sensor is attached to the ${(i-1)}$--segment. 
Orientation data from the $i$--sensor, recorded during two one-axis functional movements of the $i$--segment and an N-pose, are used to calculate two axes of segment rotation through Principal Component Analysis (PCA). The N-pose data also help to determine the gravity vector observed by the $i$--sensor. Then, four options of $^{Seg(i)}q_{S(i)}$ are obtained using the two rotation axes and the gravity vector through orthonormalization via cross products. An average of these four options, the misalignment between the ${(i-1)}$--sensor and the ${(i-1)}$--segment ($^{Seg(i-1)}q_{S(i-1)}$), and orientation data from both sensors recorded during an N-pose, are used to obtain an initial estimation of $^{WS(i-1)}q_{WS(i)}$. The closest rotation around the $z$-axis of the sensors is used to refine this estimation. Finally, the result of $^{WS(i-1)}q_{WS(i)}$ is used to obtain a better estimation of $^{Seg(i)}q_{S(i)}$.

#### Authors: Sergio A. Salinas, Mahshad Berjis, Katarina Grolinger, and Ana Luisa Trejos
#### Western University, London, ON, Canada

## Reference
S. A. Salinas, M. Berjis, K. Grolinger, and A. L. Trejos, Estimating Thorax and Shoulder Motion Using Magnetic-Free Quaternion-Based Functional Sensor-to-Segment Calibration, International Conference on Rehabilitation Robotics (ICORR), Chicago, USA, May 2025.

