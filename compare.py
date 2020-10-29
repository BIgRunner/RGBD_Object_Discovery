#/usr/bin/env python3
# -*- coding: utf-8 -*-


'''
Author: BIgRunner

'''


import cv2
import os
import numpy as np
from collections import defaultdict
import argparse


def evaluate(gt_path, ms_path, all, one, thresh, save):

  write_file = save

  total_gt = 0
  total_correct = 0
  total_over_seg = 0
  total_under_seg = 0
  total_miss = 0
  total_noise = 0 
  total_images = 0
  total_avg_correct_measure = np.zeros(3)
  total_avg_over_measure = np.zeros(3)
  total_avg_under_measure = np.zeros(3)
  total_avg_co_measure = np.zeros(3)
  total_cdr = 0.0
  total_odr = 0.0
  total_codr = 0.0

  files = list()
  if all:
    files=os.listdir(gt_path)
  else:
    files.append(one)
  
  for file_name in files:

    # Load ground truth and your results 
    gt_file = os.path.join(gt_path, file_name)
    ms_file = os.path.join(ms_path, file_name.replace("GT", "MS").replace("gt", "ms"))
    if not os.path.isfile(gt_file) or not os.path.isfile(ms_file):
      print("File {} doesn't exist".format(file_name))
      continue

    total_images += 1

    gt_planes = cv2.imread(gt_file, cv2.IMREAD_UNCHANGED)
    ms_planes = cv2.imread(ms_file, cv2.IMREAD_UNCHANGED)
    
    planes_in_gt = gt_planes.max()
    planes_in_ms = ms_planes.max()

    # Compute overlap tables
    overlap_table = np.zeros((planes_in_ms, planes_in_gt), dtype=np.uint)
    ms_planes_size = np.zeros(planes_in_ms, dtype=np.uint)
    gt_planes_size = np.zeros(planes_in_gt, dtype=np.uint)

    for r in range(gt_planes.shape[0]):
      for c in range(gt_planes.shape[1]):
        if gt_planes[r,c] != 0:
          gt_planes_size[gt_planes[r,c]-1] += 1
        if ms_planes[r,c] != 0:
          ms_planes_size[ms_planes[r,c]-1] += 1
        if gt_planes[r,c] !=0 and ms_planes[r,c]!=0:
          overlap_table[ms_planes[r,c]-1][gt_planes[r,c]-1] +=1

    # Compute segmented regions into correct, oversegmentation, and undersegmentation

    # initialize 
    gt_region_classified = np.zeros(planes_in_gt, dtype=np.uint8)
    ms_region_classified = np.zeros(planes_in_ms, dtype=np.uint8)
    gt_region_mapping = defaultdict(lambda: False)
    ms_region_mapping = defaultdict(lambda: False)
    gt_mapping_measures = np.zeros((planes_in_gt, 3))
    ms_mapping_measures = np.zeros((planes_in_ms, 3))


    # for correct case
    for ms in range(planes_in_ms):
      for gt in range(planes_in_gt):
          measure_gt = overlap_table[ms][gt]/gt_planes_size[gt]
          measure_ms = overlap_table[ms][gt]/ms_planes_size[ms]
          measure_sp = (gt_planes.size-gt_planes_size[gt])/(gt_planes.size-overlap_table[ms][gt])
          if measure_gt >= thresh and measure_ms >= thresh:
            gt_region_classified[gt] = 1
            ms_region_classified[ms] = 1
            gt_region_mapping[gt]=ms
            ms_region_mapping[ms]=gt
            gt_mapping_measures[gt][0]=measure_gt
            gt_mapping_measures[gt][1]=measure_ms
            gt_mapping_measures[gt][2]=measure_sp
            ms_mapping_measures[ms][0]=measure_gt
            ms_mapping_measures[ms][1]=measure_ms
            ms_mapping_measures[ms][2]=measure_sp

    # for over-segmentation case
    for gt in range(planes_in_gt):
      total_overlap = 0
      total_ms_region = 0
      many_ms_mapping = 0
      for ms in range(planes_in_ms):
        single_ms_measure = overlap_table[ms][gt]/ms_planes_size[ms]
        if single_ms_measure >= thresh:
          many_ms_mapping += 1
          total_overlap += overlap_table[ms][gt]
          total_ms_region += ms_planes_size[ms]
          ms_region_mapping[ms] = gt

      if many_ms_mapping > 1:
        measure_gt = total_overlap/gt_planes_size[gt]
        measure_ms = total_overlap/total_ms_region
        measure_sp = (gt_planes.size-gt_planes_size[gt])/(gt_planes.size-total_overlap)
        if measure_gt >= thresh and measure_ms >= thresh:
          this_better = True
          if not gt_region_mapping[gt]:
            if measure_gt+measure_ms <= gt_mapping_measures[gt][0]+gt_mapping_measures[gt][1]:
              this_better = False
          if this_better:
            gt_region_classified[gt] = 2
            gt_mapping_measures[gt][0]=measure_gt
            gt_mapping_measures[gt][1]=measure_ms
            print(measure_sp)
            print("measure_sp {}".format(measure_sp))
            gt_mapping_measures[gt][2]=measure_sp
            for ms in range(planes_in_ms):
              if ms_region_mapping[ms] == gt:
                ms_region_classified[ms] = 2
                ms_mapping_measures[ms][0]=measure_gt
                ms_mapping_measures[ms][1]=measure_ms
                ms_mapping_measures[ms][2]=measure_sp

    # for under-segmentation case
    for ms in range(planes_in_ms):
      total_overlap = 0
      total_gt_region = 0
      many_gt_mapping = 0
      for gt in range(planes_in_gt):
        single_gt_measure = overlap_table[ms][gt]/gt_planes_size[gt]
        if single_gt_measure >= thresh:
          many_gt_mapping += 1
          total_overlap += overlap_table[ms][gt]
          total_gt_region += gt_planes_size[gt]
          gt_region_mapping[gt] = ms
      
      if many_gt_mapping > 1:
        measure_gt = total_overlap/total_gt_region
        measure_ms = total_overlap/ms_planes_size[ms]
        measure_sp = (gt_planes.size-total_gt_region)/(gt_planes.size-total_overlap)
        if measure_gt >= thresh and measure_ms >= thresh:
          this_better=True
          if not ms_region_mapping[ms]:
            if measure_gt+measure_ms <= ms_mapping_measures[ms][0]+ms_mapping_measures[ms][1]:
              this_better = False
          if this_better:
            ms_region_classified[ms] = 3
            ms_mapping_measures[ms][0] = measure_gt
            ms_mapping_measures[ms][1] = measure_ms
            ms_mapping_measures[ms][2] = measure_sp
            for gt in range(planes_in_gt):
              if gt_region_mapping[gt] == ms:
                gt_region_classified[gt] = 3
                gt_mapping_measures[gt][0] = measure_gt
                gt_mapping_measures[gt][1] = measure_ms
                gt_mapping_measures[gt][2] = measure_sp

    
    # Output tables
    line = "Segmentation results in {}:\n".format(file_name)
    head_form = "{0:>19}{1:>14}{2:>19}{3:>12}{4:>12}{5:>12}\n"
    line += head_form.format("GT Region(s)","Category","MS region(s)","Measure_GT", "Measure_MS", "Measure_SP")
    content_form = "{0:>19}{1:>14}{2:>19}{3:12.3f}{4:12.3f}{5:12.3f}\n"

    avg_co_measure = np.zeros(3, dtype=np.float)
    # For correct case
    number_correct = 0
    avg_correct_measure = np.zeros(3, dtype=np.float)
    for gt in range(planes_in_gt):
      if gt_region_classified[gt] == 1:
        line+=content_form.format(str(gt), "correct", str(gt_region_mapping[gt]), gt_mapping_measures[gt][0], gt_mapping_measures[gt][1], gt_mapping_measures[gt][2])
        number_correct += 1
        avg_correct_measure += gt_mapping_measures[gt]
        avg_co_measure += gt_mapping_measures[gt]
    if number_correct > 0:
      avg_correct_measure = avg_correct_measure/number_correct
    line += "\n"

    # For over-segmentation case
    number_over_seg = 0
    avg_over_measure = np.zeros(3)
    for gt in range(planes_in_gt):
      if gt_region_classified[gt] == 2:
        ms_mapping_list=[]
        for ms in range(planes_in_ms):
          if ms_region_mapping[ms] == gt:
            ms_mapping_list.append(ms)
        line+=content_form.format(str(gt),"over-seg",str(ms_mapping_list)[1:-1], gt_mapping_measures[gt][0], gt_mapping_measures[gt][1], gt_mapping_measures[gt][2])
        number_over_seg += 1
        avg_over_measure += gt_mapping_measures[gt]
        avg_co_measure += gt_mapping_measures[gt]
    if number_over_seg > 0:
      avg_over_measure = avg_over_measure/number_over_seg
    line += "\n"

    avg_co_measure /= (number_correct+number_over_seg)

    # For under-segmentation case
    number_under_seg=0
    avg_under_measure = np.zeros(3)
    for ms in range(planes_in_ms):
      if ms_region_classified[ms] == 3:
        gt_mapping_list=[]
        for gt in range(planes_in_gt):
          if gt_region_mapping[gt] == ms:
            gt_mapping_list.append(gt)
        line+=content_form.format(str(gt_mapping_list)[1:-1],"under-seg",str(ms), ms_mapping_measures[ms][0], ms_mapping_measures[ms][1], ms_mapping_measures[ms][2])
        number_under_seg += 1
        avg_under_measure += ms_mapping_measures[ms]
    if number_under_seg > 0:
      avg_under_measure = avg_under_measure/number_under_seg
    line += "\n"


    # For missing case
    number_missed = 0
    for gt in range(planes_in_gt):
      if gt_region_classified[gt]==0:
        line+=content_form.format(str(gt), "missed", " ", 0.0, 0.0, 0.0)
        number_missed += 1
    line += "\n"

    # For Noise case
    number_noise = 0 
    for ms in range(planes_in_ms):
      if ms_region_classified[ms] == 0:
        line+=content_form.format(" ", "noise", str(ms), 0.0, 0.0, 0.0)
        number_noise += 1
    line += "\n"

    
    line += "\n"
    line += "Regions in Ground Truth: {}\n".format(planes_in_gt)
    line += "Regions in Machine Segmentation: {}\n".format(planes_in_ms)
    line += "\n"
    line += "CORRECTION regions: {}\n".format(number_correct)
    line += "\t Average Measure: {:.3f}, {:.3f}, {:.3f}\n".format(avg_correct_measure[0], avg_correct_measure[1], avg_correct_measure[2])

    line += "\n"
    line += "OVER-SEGMENTATION regions: {}\n".format(number_over_seg)
    line += "\t Average Measure: {:.3f}, {:.3f}\n".format(avg_over_measure[0], avg_over_measure[1], avg_over_measure[2])

    line += "\n"
    line += "UNDER-SEGMENTATION regions: {}\n".format(number_under_seg)
    line += "\t Average Measure: {:.3f}, {:.3f}\n".format(avg_under_measure[0], avg_under_measure[1], avg_under_measure[2])

    line += "\n"
    line += "MISSED regions: {}\n".format(number_missed)
    line += "NOISING regions: {}\n".format(number_noise)
    print(line)
    if write_file:
      with open(os.path.join(ms_path, file_name+".compared"), "w") as f:
        f.write(line)

    total_gt += planes_in_gt
    total_correct += number_correct
    total_over_seg += number_over_seg
    total_under_seg += number_under_seg
    total_miss += number_missed
    total_noise += number_noise
    if number_correct>0:
      total_avg_correct_measure += avg_correct_measure
      total_avg_over_measure += avg_over_measure
      total_avg_under_measure += avg_under_measure
      total_avg_co_measure += avg_co_measure
      total_cdr += number_correct/planes_in_gt
      total_odr += number_over_seg/planes_in_gt
  
  if total_images > 0:
    result_form = "{0:>12}: {1:6.3f}, {2:>12}: {3:6.3f}\n"
    results = "\n"
    results+=result_form.format("GT regions", total_gt/total_images, "Correct", total_correct/total_images)
    results+=result_form.format("OverSeg", total_over_seg/total_images, "UnderSeg", total_under_seg/total_images)
    results+=result_form.format("Missed", total_miss/total_images, "Noise", total_noise/total_images)
    results+="{:>12}{:>12}{:>12}{:>12}\n".format("Type", "Sensitivity", "Specificity", "DR")
    measure_form = "{:>16}{:12.3f}{:12.3f}{:12.3f}\n"
    total_avg_correct_measure /= total_images
    total_avg_over_measure /= total_images
    total_avg_under_measure /= total_images
    total_avg_co_measure /= total_images
    results+=measure_form.format("correct", total_avg_correct_measure[0], total_avg_correct_measure[2], total_cdr/total_images)
    results+=measure_form.format("over segment", total_avg_over_measure[0], total_avg_over_measure[2], total_odr/total_images)
    results+=measure_form.format("correct and over", total_avg_co_measure[0], total_avg_co_measure[2], (total_cdr+total_odr)/total_images) 
    print(results)
    with open(os.path.join(ms_path,"average.result"), "w") as f:
        f.write(results)


if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description="Compare plane segmentation results with ground truth")
  group = parser.add_mutually_exclusive_group()
  parser.add_argument("gt_path", type=str, help="Directory of ground truth segmentation results")
  parser.add_argument("ms_path", type=str, help="Directory of compared segmentation results")
  group.add_argument("--all", action="store_true", help="compare all files")
  group.add_argument("--one", type=str, help="file basename, e.g. abw_3 will compare abw_3_gt.png with abw_3_ms.png")
  parser.add_argument("-t", "--thresh", type=float, default=0.51, help="Segmenation threshold")
  parser.add_argument("-s", "--save", action="store_true", help="If set, write evalution result into files")
  args = parser.parse_args()

  assert os.path.isdir(args.gt_path), "Ground truth directory doesn't exists!"
  assert os.path.isdir(args.ms_path), "Segmentation results directory doesn't exists!"
  assert 0.5<=args.thresh<=1, "Segmentation threshold should be set in (0.5, 1)!"

  evaluate(**vars(args))
  
