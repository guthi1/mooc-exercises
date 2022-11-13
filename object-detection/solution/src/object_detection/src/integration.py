#!/usr/bin/env python
# coding: utf-8

<<<<<<< HEAD
# In[1]:
=======
# In[3]:
>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)


def DT_TOKEN():
    # todo change this to your duckietown token
<<<<<<< HEAD
    dt_token = "dt1-3nT8KSoxVh4Migd7N6Nsjy5q8BHtzjcsyz57DKmyDpra2Wp-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfj8ef646qjC1sdUCLLvJDADTr2CFxshUaw"
=======
<<<<<<< HEAD
    dt_token = "dt1-3nT8KSoxVh4MdLnE1Bq2mTkhRpbR35G8mmbjExKF7zGm6g4-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfYtfGWrfSxeihNZvYVNfNmnCBP28LeqDxL"
=======
    dt_token = "dt1-3nT8KSoxVh4Migd7N6Nsjy5q8BHtzjcsyz57DKmyDpra2Wp-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfj8ef646qjC1sdUCLLvJDADTr2CFxshUaw"
>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)
>>>>>>> da0aeb9 (Completing rebase by removing yolo weights)
    return dt_token

def MODEL_NAME():
    # todo change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5"
    return "yolov5"

<<<<<<< HEAD
=======

>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)
# In[2]:


def NUMBER_FRAMES_SKIPPED():
    # todo change this number to drop more frames
    # (must be a positive integer)
    return 0

<<<<<<< HEAD
=======

>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)
# In[3]:


# `class` is the class of a prediction
def filter_by_classes(clas):
    # Right now, this returns True for every object's class
    # Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    return True

<<<<<<< HEAD
=======

>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)
# In[4]:


# `scor` is the confidence score of a prediction
def filter_by_scores(scor):
    # Right now, this returns True for every object's confidence
    # Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    return True

<<<<<<< HEAD
=======

>>>>>>> 24d4846 (Adding gitignore rules to not push nn_models and dataset)
# In[5]:


# `bbox` is the bounding box of a prediction, in xyxy format
# So it is of the shape (leftmost x pixel, topmost y pixel, rightmost x pixel, bottommost y pixel)
def filter_by_bboxes(bbox):
    # Like in the other cases, return False if the bbox should not be considered.
    return True

