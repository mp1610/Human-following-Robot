# Human-following-Robot

This thesis study investigates the development of a human-following robot virtually using ROS, PyTorch, and Gazebo.

The code attached contains the different ROS commands, Gazebo worlds, and YOLOv3 trained models. To reproduce this project, follow the instructions in the Appendix of the report.

All the detail of the project methodology is included in the report.

Brief Summary:

Firstly, an explanation is given into the creation of the conda virtual environment, justifying
any relevant packages or versions which have been used. Secondly, since the chosen robot
that will be used in this project is the TurtleBot3 Waffle Pi, a description of its key features will
be listed to understand its functionality and capabilities prior to any training or application of
the robot. After this, different Gazebo scenarios will be discussed, within which a human model
walks in a particular pathway, as coded within the Gazebo custom worlds xml file.

Now that the base of the design is complete, custom human image data within Gazebo is
collected, after which the images are augmented and modified in order to fit the compatible
COCO format that can then be used in the model training process. PyTorch’s MMDetection is
used to apply the relevant algorithm (YOLOv3) after consideration between two different
algorithms. After creating the YOLOv3 configuration file, a model can be trained based on the
image dataset. An explanation is given into how the testing of the model is carried out,
particularly what the results show and how they give an insight into the quality of the model.
Finally, the trained detection model is implemented into an ROS code that applies into the
TurtleBot3 robot and directs the robot to follow the human throughout a range of scenarios
based on its human detection capabilities to see how well it performs in increasingly difficult
scenarios.
