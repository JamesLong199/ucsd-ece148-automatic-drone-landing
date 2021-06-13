# Automatic Drone Localization and Landing

## [Project Wiki Page](https://guitar.ucsd.edu/maeece148/index.php/2021SpringTeam5)

## Objective
Let DJI Tello Drone automatically detects and land on a 1/10 scale vehicle. An Apriltag is placed on the landing platform on the vehicle, the drone performs localization and landing based on apriltag detection.

<figure>
    <img src="https://user-images.githubusercontent.com/66498825/121795668-bf19c680-cbc7-11eb-937f-a026e0865233.jpg" alt="tello_img" width="400">
</figure>

## Provided Functionality
1. Keyboard drone motion control
2. Capture and record video
3. Activate automatic landing process

## Software Dependency
1. [TelloPy](https://github.com/hanyazou/TelloPy)
2. PyAV
3. Pygame
4. Pupil_apriltags
5. OpenCV

## Keyboard Manual Control
* Tab                          -- Takeoff
* Backspace                    -- Land
* W/S/A/D                      -- Forward/Backward/Left/Right
* Left/Right/Up/Down           -- Counterclockwise/Clockwise/Up/Down
* Enter                        -- Take picture (picture taken at key-release)
* R                            -- Record video (Press and hold during recording)
* L                            -- Activate/Deactivate automatic landing process

## Apriltag Localization Algorithm
Check [this project](https://github.com/JamesLong199/Autonomous-Transducer-Project/tree/main/localization) for detailed explanation and experiment results. 

## Automatic Landing Algorithm
The automatic landing algorithm is fine-tuned empirically. Hyperparameter that affects its performance includes
* Tolerance of (x,y,z) position alignment with apriltag tag center
* Tolerance of drone angle alignment with global coordinate system
* Tolerance of (x,y) position alignment between drone image center and tag center in the image
* Landing zone range
* Low speed zone range
* Drone speed

## [Demo Video](https://youtu.be/gSxaIQHgp4U)





