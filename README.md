# Computer Vision Project
---

This is a computer vision project done for Ain Shams University Fall 2022 session, it is assumed that the person running the code has already installed the simulator on his device (that runs linux). This file aids with setting up an anaconda environment to run the code provided in this repo.

The README shows how to:
1. set up the conda environment
2. overview of the files in the repo and how to run

# Set up an anaconda environment
---
1. Install anaconda [from this link](https://www.anaconda.com/products/distribution)

After running the installer you will have a base environment on your computer, clone this base envrionment into a new one and name it anything you like (say, vision)

2. run conda create --name vision --clone base

After running this command you now have a vision environment that has all of conda's default packages.

3. To Get into the vision environment, run conda activate vision

Now we need to install some additional packages

4. Install additional packages using the syntax conda install -c \<source\> \<package-name\> to check the conda package source use [anaconda.org](https://anaconda.org/)
list of needed packages:
- opencv
- python-socketio (version 4.6.1)
- python-engineio (version 3.13.2)
- eventlet

# Run The Code

to run the code simply move to the code folder (cd code) and run the python file drive_rover.py (python3 drive_rover.py)

# Overview of files

1. code folder
- driver_rover.py: is the main file of the program, it sends the commands to the simulator and calls perception and decision.  
- perception.py: is the file that performs computer vision and image processing techniques on the Rover's image.  
- decision.py: is the file that takes a decision on steering and throttle based on the perception step.  

2. project_pipline.ipynb: is a jupyter notebook that shows the project's pipline and is used mainly for testing images before editing the code.  

3. test_dataset: is a folder that has many test images from the simulator.  

4. calibration_images: is a folder that has an image with grids and an image with a rock for calibration purposes.   

