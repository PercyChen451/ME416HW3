#!/usr/bin/env python3
import cv2
import numpy as np
import os

def create_dataset():
    # Define paths to input images
    train_img_path = 'line-train.png'
    cross_img_path = 'line-cross.png'
    test_img_path = 'line-test.png'
    
    # Define output paths
    output_files = [
        'train_positive.png', 'train_negative.png',
        'cross_positive.png', 'cross_negative.png',
        'test_positive.png', 'test_negative.png'
    ]
    
    # Load images
    train_img = cv2.imread(train_img_path)
    cross_img = cv2.imread(cross_img_path)
    test_img = cv2.imread(test_img_path)
    
    # Define regions of interest (ROIs) for positive and negative samples
    # These coordinates need to be adjusted based on your specific images
    train_pos_roi = (100, 150, 200, 250)  # (x1, y1, x2, y2) - line region
    train_neg_roi = (300, 400, 400, 500)   # (x1, y1, x2, y2) - background region
    
    cross_pos_roi = (120, 170, 220, 270)
    cross_neg_roi = (320, 420, 420, 520)
    
    test_pos_roi = (110, 160, 210, 260)
    test_neg_roi = (310, 410, 410, 510)
    
    # Crop and save images
    # Training set
    train_pos = train_img[train_pos_roi[1]:train_pos_roi[3], train_pos_roi[0]:train_pos_roi[2]]
    train_neg = train_img[train_neg_roi[1]:train_neg_roi[3], train_neg_roi[0]:train_neg_roi[2]]
    
    # Cross-validation set
    cross_pos = cross_img[cross_pos_roi[1]:cross_pos_roi[3], cross_pos_roi[0]:cross_pos_roi[2]]
    cross_neg = cross_img[cross_neg_roi[1]:cross_neg_roi[3], cross_neg_roi[0]:cross_neg_roi[2]]
    
    # Testing set
    test_pos = test_img[test_pos_roi[1]:test_pos_roi[3], test_pos_roi[0]:test_pos_roi[2]]
    test_neg = test_img[test_neg_roi[1]:test_neg_roi[3], test_neg_roi[0]:test_neg_roi[2]]
    
    # Convert to HSV color space (optional but recommended)
    train_pos_hsv = cv2.cvtColor(train_pos, cv2.COLOR_BGR2HSV)
    train_neg_hsv = cv2.cvtColor(train_neg, cv2.COLOR_BGR2HSV)
    cross_pos_hsv = cv2.cvtColor(cross_pos, cv2.COLOR_BGR2HSV)
    cross_neg_hsv = cv2.cvtColor(cross_neg, cv2.COLOR_BGR2HSV)
    test_pos_hsv = cv2.cvtColor(test_pos, cv2.COLOR_BGR2HSV)
    test_neg_hsv = cv2.cvtColor(test_neg, cv2.COLOR_BGR2HSV)
    
    # Save images
    cv2.imwrite(output_files[0], train_pos_hsv)
    cv2.imwrite(output_files[1], train_neg_hsv)
    cv2.imwrite(output_files[2], cross_pos_hsv)
    cv2.imwrite(output_files[3], cross_neg_hsv)
    cv2.imwrite(output_files[4], test_pos_hsv)
    cv2.imwrite(output_files[5], test_neg_hsv)
    
    print("Dataset created successfully!")

if __name__ == '__main__':
    create_dataset()