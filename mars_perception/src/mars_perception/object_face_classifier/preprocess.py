from typing import *
import random
import yaml
import argparse
import os
import numpy as np
import cv2

from mars_perception.object_face_classifier.util import obj_center_crop, measure_darkness, rotate_image

DATA_DIR = './data'

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-cfg_path", help="config file", type=str, default='./config.yml')
    parser.add_argument("-show", help="show images", action='store_true')

    args = vars(parser.parse_args())
    with open(args['cfg_path'], "r") as f:
        p = yaml.safe_load(f)
    params = p['dataset']

    random.seed(p['seed'])

    dataset_path = os.path.join(DATA_DIR,params['dataset_name'])

    proc_dataset_name = params['dataset_name'] + '_proc'
    proc_dataset_path = os.path.join(DATA_DIR,proc_dataset_name)
    if not os.path.exists(proc_dataset_path):
        os.mkdir(proc_dataset_path)

    min_feats = np.inf
    cnt = 0
    for (dirpath, dirnames, filenames) in os.walk(dataset_path):
        label = dirpath.split('/')[-1]
        if not os.path.exists(f'{proc_dataset_path}/{label}') and len(filenames) > 0:
            os.mkdir(f'{proc_dataset_path}/{label}')
        for file in filenames:
            print(f'{dirpath}/{file}')
            im = cv2.imread(f'{dirpath}/{file}')
            im = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
            im = obj_center_crop(im,params['img_dim'])
            feats = len(np.argwhere(im != 0))
            min_feats = min(min_feats,feats)
            print(feats)
            if(feats > params['min_features']):
                if args['show']:
                    cv2.imshow('debug',im)
                    cv2.waitKey(0)
                rot_im = rotate_image(im,random.uniform(*params['rand_rotate'])) 
                cv2.imwrite(f'{proc_dataset_path}/{label}/{cnt}.jpg',im)
                cnt += 1
                cv2.imwrite(f'{proc_dataset_path}/{label}/{cnt}.jpg',rot_im)
                cnt += 1
    
        
    print(f'preprocessed dataset size: {cnt}')
                